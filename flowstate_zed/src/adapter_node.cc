// Copyright 2026 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "flowstate_zed/adapter_node.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "flowstate_zed/camera_discovery.h"
#include "flowstate_zed/image_conversion.h"
#include "flowstate_zed/zed_parameters.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sl/Camera.hpp"
#include "snapshot_interfaces/msg/image_snapshot.hpp"
#include "std_msgs/msg/header.hpp"
#include "zed_components/zed_camera_component.hpp"
#include "zed_components/zed_camera_one_component.hpp"

namespace flowstate_zed {
namespace {

constexpr absl::Duration kImageDeadline = absl::Seconds(5);
constexpr absl::Duration kStartupDeadline = absl::Seconds(8);
// A camera silent for this long ends the adapter; the spawner recreates it.
constexpr std::chrono::seconds kLivenessTimeout(15);
constexpr std::chrono::seconds kLivenessPollPeriod(1);
constexpr std::chrono::milliseconds kSubExecutorSpinSlice(100);
constexpr std::chrono::milliseconds kWrapperStartupSpinSlice(20);
constexpr std::chrono::milliseconds kWrapperSteadySpinSlice(100);
constexpr size_t kServiceExecutorThreads = 3;
constexpr size_t kSubscriptionQueueDepth = 5;

void CancelQuietly(rclcpp::Executor& executor) {
  try {
    executor.cancel();
  } catch (const std::exception&) {
  }
}

// Opens the stereo camera through the ZED SDK, reads the factory unrectified
// stereo transform in IMAGE/METER coordinates (right sensor in left sensor's
// frame), and closes the camera. The camera must not be open elsewhere.
absl::StatusOr<geometry_msgs::msg::Transform> ReadRightSensorPose(
    uint32_t serial_number) {
  sl::InitParameters init;
  init.input.setFromSerialNumber(serial_number);
  init.camera_resolution = sl::RESOLUTION::HD1200;
  init.depth_mode = sl::DEPTH_MODE::NONE;
  init.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
  init.coordinate_units = sl::UNIT::METER;
  // Self-calibration changes the raw rotation at every open; off here and in
  // the wrapper so Describe always reports factory calibration.
  init.camera_disable_self_calib = true;

  sl::Camera camera;
  const sl::ERROR_CODE error = camera.open(init);
  if (error != sl::ERROR_CODE::SUCCESS) {
    return absl::UnavailableError(absl::StrCat("ZED SDK could not open camera ",
                                               serial_number, ": ",
                                               sl::toString(error).c_str()));
  }
  const sl::Transform transform =
      camera.getCameraInformation()
          .camera_configuration.calibration_parameters_raw.stereo_transform;
  camera.close();

  const sl::Translation translation = transform.getTranslation();
  const sl::Orientation orientation = transform.getOrientation();
  geometry_msgs::msg::Transform pose;
  pose.translation.x = translation.x;
  pose.translation.y = translation.y;
  pose.translation.z = translation.z;
  pose.rotation.x = orientation.x;
  pose.rotation.y = orientation.y;
  pose.rotation.z = orientation.z;
  pose.rotation.w = orientation.w;
  return pose;
}

absl::Status ConvertStampedMatToRgb8(stereolabs::StampedSlMat& stamped,
                                     sensor_msgs::msg::Image& dst) {
  size_t channels = 0;
  const uint8_t* ptr = nullptr;
  switch (stamped.mat.getDataType()) {
    case sl::MAT_TYPE::U8_C3:
      channels = 3;
      ptr = reinterpret_cast<const uint8_t*>(
          stamped.mat.getPtr<sl::uchar3>(sl::MEM::CPU));
      break;
    case sl::MAT_TYPE::U8_C4:
      channels = 4;
      ptr = reinterpret_cast<const uint8_t*>(
          stamped.mat.getPtr<sl::uchar4>(sl::MEM::CPU));
      break;
    default:
      return absl::InvalidArgumentError(
          absl::StrCat("Unsupported sl::MAT_TYPE: ",
                       static_cast<int>(stamped.mat.getDataType())));
  }
  const uint32_t width = static_cast<uint32_t>(stamped.mat.getWidth());
  const uint32_t height = static_cast<uint32_t>(stamped.mat.getHeight());
  const uint32_t step = static_cast<uint32_t>(stamped.mat.getStepBytes());
  std_msgs::msg::Header header;
  header.stamp = stamped.stamp;
  header.frame_id = stamped.frame_id;
  return ConvertBgrToRgb8(ptr, width, height, step,
                          static_cast<size_t>(step) * height, channels, header,
                          dst);
}

}  // namespace

AdapterNode::AdapterNode(const DiscoveredCamera& camera)
    : flowstate_common::BaseAdapterNode(
          camera.serial, {}, camera.driver_type,
          rclcpp::NodeOptions().use_intra_process_comms(true)),
      camera_(camera),
      cache_(camera.stereo) {
  DeclareParameters();
  Subscribe();
  CreateFlowstateServices();
  StartExecutorThread();
}

AdapterNode::~AdapterNode() {
  // Main() uses members of this class, so it must end before they are
  // destroyed. The base class destructor would join too late.
  cache_.RequestStop();
  {
    absl::MutexLock lock(&executor_mutex_);
    if (active_executor_ != nullptr) {
      CancelQuietly(*active_executor_);
    }
  }
  if (thread_.joinable()) {
    thread_.join();
  }
}

void AdapterNode::DeclareParameters() {
  DeclareAdapterParameters(*this);
  on_set_handle_ = add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter>& parameters) {
        return ValidateCameraParameters(parameters);
      });
  post_set_handle_ = add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters) {
        PostSetParameters(parameters);
      });
}

void AdapterNode::Subscribe() {
  subscription_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::SubscriptionOptions options;
  options.callback_group = subscription_group_;

  left_image_sub_ = create_subscription<SlImageAdapter>(
      ColorImageTopic(), kSubscriptionQueueDepth,
      [this](std::unique_ptr<stereolabs::StampedSlMat> msg) {
        const rclcpp::Time stamp = msg->stamp;
        cache_.OnLeftImage(std::move(msg), stamp, get_clock()->now());
      },
      options);
  color_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      InfoTopic(ColorImageTopic()), kSubscriptionQueueDepth,
      [this](sensor_msgs::msg::CameraInfo::UniquePtr msg) {
        cache_.OnLeftInfo(std::move(msg));
      },
      options);
  if (!camera_.stereo) return;

  const std::string right_topic = RightImageTopic(serial_);
  right_image_sub_ = create_subscription<SlImageAdapter>(
      right_topic, kSubscriptionQueueDepth,
      [this](std::unique_ptr<stereolabs::StampedSlMat> msg) {
        const rclcpp::Time stamp = msg->stamp;
        cache_.OnRightImage(std::move(msg), stamp);
      },
      options);
  right_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      InfoTopic(right_topic), kSubscriptionQueueDepth,
      [this](sensor_msgs::msg::CameraInfo::UniquePtr msg) {
        cache_.OnRightInfo(std::move(msg));
      },
      options);
}

std::string AdapterNode::ColorImageTopic() const {
  return flowstate_zed::ColorImageTopic(serial_, camera_.stereo);
}

absl::Status AdapterNode::Main() {
  if (camera_.stereo) {
    // Before the wrapper opens the camera: only one handle can hold it.
    absl::StatusOr<geometry_msgs::msg::Transform> pose =
        ReadRightSensorPose(camera_.serial_number);
    if (!pose.ok()) {
      return pose.status();
    }
    right_pose_ = *pose;
  }

  if (cache_.IsStopRequested() || !rclcpp::ok()) {
    return absl::OkStatus();
  }
  if (absl::Status start_status = StartWrapper(); !start_status.ok()) {
    return rclcpp::ok() ? start_status : absl::OkStatus();
  }

  absl::Status status = absl::OkStatus();
  // Everything below throws once rclcpp is shut down, which can happen while
  // this adapter is still starting.
  try {
    cache_.ResetLivenessClock(get_clock()->now());
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                      kServiceExecutorThreads);
    rclcpp::executors::SingleThreadedExecutor sub_executor;
    sub_executor.add_callback_group(subscription_group_,
                                    get_node_base_interface());
    std::atomic<bool> sub_stop{false};

    // Dedicated callback group so a blocking fps restart in PostSetParameters()
    // cannot starve liveness or stop checks.
    timer_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    liveness_timer_ = create_wall_timer(
        kLivenessPollPeriod,
        [this, &executor, &sub_executor, &sub_stop]() {
          rclcpp::Duration silent_for(0, 0);
          if (cache_.IsSilent(get_clock()->now(),
                              rclcpp::Duration(kLivenessTimeout),
                              &silent_for)) {
            RCLCPP_ERROR(get_logger(), "No image for %.0f s. Stopping adapter.",
                         silent_for.seconds());
            cache_.RequestStop();
          }
          if (cache_.IsStopRequested()) {
            sub_stop = true;
            CancelQuietly(executor);
            CancelQuietly(sub_executor);
          }
        },
        timer_group_);

    executor.add_node(get_node_base_interface());
    std::thread sub_thread([this, &sub_executor, &sub_stop]() {
      try {
        while (rclcpp::ok() && !sub_stop) {
          sub_executor.spin_once(kSubExecutorSpinSlice);
        }
      } catch (const std::exception& e) {
        if (rclcpp::ok()) {
          RCLCPP_ERROR(get_logger(), "Subscription executor failed: %s",
                       e.what());
        }
      }
    });

    bool should_spin = false;
    {
      absl::MutexLock lock(&executor_mutex_);
      if (!cache_.IsStopRequested()) {
        active_executor_ = &executor;
        should_spin = true;
      }
    }
    if (should_spin) {
      try {
        executor.spin();
      } catch (const std::exception& e) {
        if (rclcpp::ok()) {
          status =
              absl::InternalError(absl::StrCat("Executor failed: ", e.what()));
        }
      }
    }
    {
      absl::MutexLock lock(&executor_mutex_);
      active_executor_ = nullptr;
    }

    sub_stop = true;
    CancelQuietly(sub_executor);
    if (sub_thread.joinable()) {
      sub_thread.join();
    }
    liveness_timer_.reset();
  } catch (const std::exception& e) {
    if (rclcpp::ok()) {
      status = absl::InternalError(absl::StrCat("Executor failed: ", e.what()));
    }
  }
  StopWrapper();
  return status;
}

absl::Status AdapterNode::StartWrapper() {
  // Snapshot adapter parameters before locking wrapper_mutex_ so we preserve
  // the strict lock order NodeParameters::mutex_ -> wrapper_mutex_.
  const CameraSettings settings = ReadCameraSettings(*this);
  bool trim_auto_range_max = false;
  {
    absl::MutexLock pending_lock(&pending_mutex_);
    wrapper_accepting_sync_ = false;
    stereo_auto_range_.last_auto_exposure = settings.auto_exposure;
    trim_auto_range_max = stereo_auto_range_.trim_auto_range_max;
  }

  absl::MutexLock lock(&wrapper_mutex_);
  try {
    wrapper_fps_ = settings.fps;
    const rclcpp::NodeOptions options =
        BuildWrapperOptions(camera_, settings, trim_auto_range_max);
    if (camera_.stereo) {
      wrapper_ = std::make_shared<stereolabs::ZedCamera>(options);
    } else {
      wrapper_ = std::make_shared<stereolabs::ZedCameraOne>(options);
    }
    wrapper_stop_ = false;
    auto wrapper_executor =
        std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    wrapper_executor->add_node(wrapper_->get_node_base_interface());
    // Spawn last so a failure above leaves no thread behind. Moving
    // `wrapper_executor` into the thread ensures it is destroyed on thread
    // exit, before StopWrapper() resets `wrapper_`.
    wrapper_thread_ = std::thread([this, executor = std::move(wrapper_executor),
                                   wrapper = wrapper_.get()]() {
      bool ready = false;
      try {
        while (!wrapper_stop_ && rclcpp::ok()) {
          executor->spin_once(ready ? kWrapperSteadySpinSlice
                                    : kWrapperStartupSpinSlice);
          // initNode() runs on this executor, declares parameters, opens the
          // camera, and registers its parameter callback last. Seeing
          // kWrapperFrameRate means initNode() returned; HasFirstFrame() means
          // the camera opened and delivered a valid frame. Drain any settings
          // written during startup before flipping wrapper_accepting_sync_ and
          // advertising readiness.
          if (!ready &&
              wrapper->has_parameter(std::string(kWrapperFrameRate)) &&
              cache_.HasFirstFrame()) {
            for (;;) {
              if (wrapper_stop_ || !rclcpp::ok()) break;
              std::optional<CameraSettings> pending;
              {
                absl::MutexLock pending_lock(&pending_mutex_);
                if (!pending_settings_) {
                  wrapper_accepting_sync_ = true;
                  break;
                }
                pending = std::exchange(pending_settings_, std::nullopt);
              }
              ApplyVideoSettingsToWrapper(*wrapper, *pending);
            }
            if (!wrapper_stop_ && rclcpp::ok()) {
              cache_.SetWrapperReady(true);
              ready = true;
            }
          }
        }
      } catch (const std::exception& e) {
        if (rclcpp::ok()) {
          RCLCPP_ERROR(get_logger(), "Wrapper executor failed: %s", e.what());
        }
      }
    });
    return absl::OkStatus();
  } catch (const std::exception& e) {
    wrapper_.reset();
    return absl::InternalError(
        absl::StrCat("Wrapper start failed: ", e.what()));
  }
}

void AdapterNode::StopWrapper() {
  absl::MutexLock lock(&wrapper_mutex_);
  wrapper_stop_ = true;
  if (wrapper_thread_.joinable()) {
    wrapper_thread_.join();
  }
  cache_.SetWrapperReady(false);
  {
    absl::MutexLock pending_lock(&pending_mutex_);
    wrapper_accepting_sync_ = false;
    pending_settings_.reset();
  }
  // The wrapper's destructor stops its threads and closes the camera.
  wrapper_.reset();
}

absl::StatusOr<snapshot_interfaces::srv::Describe::Response>
AdapterNode::BuildDescribeResponse() {
  absl::StatusOr<FrameCache<stereolabs::StampedSlMat>::DescribeData> info =
      cache_.WaitForCameraInfo(kStartupDeadline);
  if (!info.ok()) {
    return info.status();
  }
  snapshot_interfaces::srv::Describe::Response response;
  if (!camera_.stereo) {
    response.sensors.push_back(
        BuildSensorInformation(info->left_info, "rgb", ColorImageTopic()));
    return response;
  }
  // Order matters: Flowstate numbers sensors from 1 in this order.
  response.sensors.push_back(
      BuildSensorInformation(info->left_info, "left", ColorImageTopic()));
  geometry_msgs::msg::TransformStamped camera_t_right;
  camera_t_right.transform = right_pose_;
  response.sensors.push_back(BuildSensorInformation(
      *info->right_info, "right", RightImageTopic(serial_), camera_t_right));
  return response;
}

absl::StatusOr<snapshot_interfaces::srv::Snapshot::Response>
AdapterNode::BuildSnapshotResponse() {
  absl::StatusOr<FrameCache<stereolabs::StampedSlMat>::SnapshotData> snap =
      cache_.WaitAndTakeSnapshot(kImageDeadline, kStartupDeadline);
  if (!snap.ok()) {
    return snap.status();
  }

  snapshot_interfaces::srv::Snapshot::Response response;
  snapshot_interfaces::msg::ImageSnapshot left_snapshot;
  left_snapshot.topic_name = ColorImageTopic();
  left_snapshot.camera_info = std::move(snap->left_info);
  if (absl::Status status =
          ConvertStampedMatToRgb8(*snap->left_frame, left_snapshot.image);
      !status.ok()) {
    return status;
  }
  response.images.push_back(std::move(left_snapshot));
  if (!camera_.stereo) {
    return response;
  }

  snapshot_interfaces::msg::ImageSnapshot right_snapshot;
  right_snapshot.topic_name = RightImageTopic(serial_);
  right_snapshot.camera_info = std::move(snap->right_info);
  if (absl::Status status =
          ConvertStampedMatToRgb8(*snap->right_frame, right_snapshot.image);
      !status.ok()) {
    return status;
  }
  response.images.push_back(std::move(right_snapshot));
  return response;
}

void AdapterNode::PostSetParameters(
    const std::vector<rclcpp::Parameter>& parameters) {
  bool restart = HasParameter(parameters, "fps");
  if (restart) {
    const int64_t target_fps = std::llround(get_parameter("fps").as_double());
    absl::MutexLock lock(&wrapper_mutex_);
    restart = !wrapper_ ||
              RequiresWrapperRestart(parameters, wrapper_fps_, target_fps);
  }
  if (restart) {
    // Frame rate is read once at camera open: recreate the wrapper.
    RCLCPP_INFO(get_logger(), "Restarting camera for fps change");
    cache_.BeginRestart();
    StopWrapper();
    if (!cache_.IsStopRequested() && rclcpp::ok()) {
      if (absl::Status start_status = StartWrapper();
          !start_status.ok() && rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "%s",
                     std::string(start_status.message()).c_str());
      }
    }
    cache_.EndRestart(get_clock()->now());
    return;
  }
  const CameraSettings settings = ReadCameraSettings(*this);
  bool sync_now = false;
  {
    absl::MutexLock pending_lock(&pending_mutex_);
    if (wrapper_accepting_sync_) {
      sync_now = true;
    } else {
      pending_settings_ = settings;
    }
  }
  if (sync_now) {
    SyncWrapperParameters();
  }
}

void AdapterNode::SyncWrapperParameters() {
  const CameraSettings settings = ReadCameraSettings(*this);
  absl::MutexLock lock(&wrapper_mutex_);
  {
    absl::MutexLock pending_lock(&pending_mutex_);
    if (!wrapper_ || !wrapper_accepting_sync_) {
      pending_settings_ = settings;
      return;
    }
  }
  ApplyVideoSettingsToWrapper(*wrapper_, settings);
}

void AdapterNode::ApplyVideoSettingsToWrapper(rclcpp::Node& wrapper,
                                              const CameraSettings& settings) {
  bool trim_auto_range_max = false;
  if (camera_.stereo) {
    absl::MutexLock pending_lock(&pending_mutex_);
    trim_auto_range_max = stereo_auto_range_.Advance(settings.auto_exposure);
  }
  const std::vector<rclcpp::Parameter> desired = BuildWrapperVideoParameters(
      settings, camera_.stereo, trim_auto_range_max);
  try {
    const std::vector<rclcpp::Parameter> changes =
        DiffWrapperParameters(desired, [&wrapper](const std::string& name) {
          return wrapper.get_parameter(name).get_parameter_value();
        });
    if (changes.empty()) return;
    const rcl_interfaces::msg::SetParametersResult result =
        wrapper.set_parameters_atomically(changes);
    if (!result.successful) {
      RCLCPP_ERROR(get_logger(), "Wrapper rejected the video settings: %s",
                   result.reason.c_str());
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(),
                 "Setting the video parameters on the wrapper failed: %s",
                 e.what());
  }
}

}  // namespace flowstate_zed
