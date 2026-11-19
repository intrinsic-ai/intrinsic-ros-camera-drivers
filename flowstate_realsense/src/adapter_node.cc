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

#include "flowstate_realsense/adapter_node.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "ament_index_cpp/get_resource.hpp"
#include "class_loader/class_loader.hpp"
#include "flowstate_realsense/helpers.h"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/parameter_service.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"
#include "rclcpp_components/node_instance_wrapper.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "snapshot_interfaces/msg/image_snapshot.hpp"
#include "snapshot_interfaces/msg/sensor_info.hpp"
#include "snapshot_interfaces/srv/describe.hpp"
#include "snapshot_interfaces/srv/snapshot.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace flowstate_realsense {
namespace {

using ::sensor_msgs::msg::CameraInfo;
using ::sensor_msgs::msg::Image;
using ::snapshot_interfaces::msg::ImageSnapshot;
using ::snapshot_interfaces::msg::SensorInfo;

// The vendor node's namespace; its name and camera_name are camera_<serial>
// (CameraName), so its topics read /realsense/camera_<serial>/<stream>/...
// and its frames camera_<serial>_<stream>_optical_frame. Each stream's frame
// is read from its images, as infra2's CameraInfo carries infra1's frame (the
// ROS stereo convention: the baseline in its P).
constexpr absl::string_view kDriverNamespace = "/realsense";
constexpr absl::string_view kDriverPackage = "realsense2_camera";
constexpr absl::string_view kDriverClass =
    "realsense2_camera::RealSenseNodeFactory";

// The four streams served on D400 color+depth cameras (D405, D455, etc.) in the
// order Describe lists them (Flowstate's sensor ids 1..4). Topics are under the
// vendor node; Y8 infrared publishes as image_rect_raw.
struct StreamSpec {
  absl::string_view sensor_name;
  absl::string_view image_topic;
  absl::string_view info_topic;
  uint32_t sensor_type;
  absl::string_view enable_parameter;
};
constexpr size_t kColor = 0;
constexpr size_t kDepth = 1;
constexpr size_t kInfra1 = 2;
constexpr size_t kInfra2 = 3;
constexpr std::array<StreamSpec, AdapterNode::kStreamCount> kStreams = {{
    {"rgb", "color/image_raw", "color/camera_info", SensorInfo::IMAGE,
     "enable_color"},
    {"depth", "aligned_depth_to_color/image_raw",
     "aligned_depth_to_color/camera_info", SensorInfo::DEPTH, "enable_depth"},
    {"ir_left", "infra1/image_rect_raw", "infra1/camera_info",
     SensorInfo::IMAGE, "enable_infra1"},
    {"ir_right", "infra2/image_rect_raw", "infra2/camera_info",
     SensorInfo::IMAGE, "enable_infra2"},
}};

// Aligned depth needs this on. The vendor's filter setup declares it after
// every sensor has declared its enable_* and *_profile parameters, so its
// presence signals that the device is open.
constexpr absl::string_view kAlignDepthParameter = "align_depth.enable";
constexpr absl::string_view kRgbColorProfileParam = "rgb_camera.color_profile";
constexpr absl::string_view kDepthColorProfileParam =
    "depth_module.color_profile";
constexpr absl::string_view kInfraProfileParam = "depth_module.infra_profile";

// A snapshot waits this long for a frame of each served stream newer than the
// request; the driver streams at 30 Hz.
constexpr absl::Duration kImageDeadline = absl::Seconds(5);
// Without a frame of a served stream for this long the node tears itself
// down; the spawner re-creates it (and the vendor node with it) on its next
// pass.
constexpr absl::Duration kLivenessTimeout = absl::Seconds(30);

geometry_msgs::msg::TransformStamped MakeIdentityTransform(
    absl::string_view parent, absl::string_view child) {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = std::string(parent);
  transform.child_frame_id = std::string(child);
  transform.transform.rotation.w = 1.0;
  return transform;
}

// Resolves the shared library path of the component `class_name` registered by
// `package` from the ament resource index, as ComponentManager does.
absl::StatusOr<std::string> ComponentLibrary(absl::string_view package,
                                             absl::string_view class_name) {
  std::string content;
  std::string prefix;
  if (!ament_index_cpp::get_resource("rclcpp_components", std::string(package),
                                     content, &prefix)) {
    return absl::NotFoundError(
        absl::StrCat("package ", package, " registers no rclcpp components"));
  }
  return ParseComponentLibrary(content, prefix, class_name);
}

}  // namespace

AdapterNode::AdapterNode(const std::string& serial)
    : flowstate_common::BaseAdapterNode(
          serial, /*locators=*/{}, "realsense",
          // The parameter services are the vendor node's (StartDriver).
          rclcpp::NodeOptions()
              .use_intra_process_comms(true)
              .start_parameter_services(false)),
      t_started_(get_clock()->now()) {
  {
    absl::MutexLock lock(&mutex_);
    t_settings_changed_ = t_started_;
    for (StreamState& stream : streams_) {
      stream.t_last_frame = t_started_;
      stream.t_last_info = t_started_;
    }
  }
  subscription_cb_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // The driver's static transforms, for the infrared extrinsics.
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  for (size_t i = 0; i < kStreamCount; ++i) {
    Subscribe(i);
  }
  CreateFlowstateServices();
  StartExecutorThread();
}

AdapterNode::~AdapterNode() {
  // Wake any snapshot waiter immediately and stop Main() before destroying
  // members it accesses.
  {
    absl::MutexLock lock(&mutex_);
    stop_requested_ = true;
  }
  executor_.cancel();
  if (thread_.joinable()) {
    thread_.join();
  }
  // Defensive cleanup if Main() never ran or exited abnormally.
  StopDriver();
}

std::vector<rclcpp::Parameter> AdapterNode::StartupParameters() const {
  // The serial keeps a leading underscore: the driver strips it, and it
  // stops an all-digit string from becoming an integer parameter. camera_name
  // prefixes the driver's frame ids, which several cameras would otherwise
  // share on /tf_static. Y8 infrared is mono8 on image_rect_raw (rectified);
  // the static transforms stay on for the extrinsics and anyone watching tf.
  return {
      rclcpp::Parameter("serial_no", absl::StrCat("_", serial_)),
      rclcpp::Parameter("camera_name", CameraName()),
      rclcpp::Parameter("enable_color", true),
      rclcpp::Parameter("enable_depth", true),
      rclcpp::Parameter("enable_infra1", true),
      rclcpp::Parameter("enable_infra2", true),
      rclcpp::Parameter("depth_module.infra1_format", std::string("Y8")),
      rclcpp::Parameter("depth_module.infra2_format", std::string("Y8")),
      rclcpp::Parameter("align_depth.enable", true),
      rclcpp::Parameter("publish_tf", true),
      rclcpp::Parameter("tf_publish_rate", 0.0),
  };
}

absl::Status AdapterNode::StartDriver() {
  absl::StatusOr<std::string> library =
      ComponentLibrary(kDriverPackage, kDriverClass);
  if (!library.ok()) return library.status();

  try {
    driver_loader_ = std::make_unique<class_loader::ClassLoader>(*library);
    const std::string factory_class = absl::StrCat(
        "rclcpp_components::NodeFactoryTemplate<", kDriverClass, ">");
    driver_factory_ =
        driver_loader_->createInstance<rclcpp_components::NodeFactory>(
            factory_class);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    options.arguments({"--ros-args", "-r",
                       absl::StrCat("__node:=", CameraName()), "-r",
                       absl::StrCat("__ns:=", kDriverNamespace)});
    options.parameter_overrides(StartupParameters());
    driver_ = std::make_unique<rclcpp_components::NodeInstanceWrapper>(
        driver_factory_->create_node_instance(options));

    // The factory derives from rclcpp::Node alone (realsense-ros built without
    // USE_LIFECYCLE_NODE, as the apt package is), so the instance is that Node:
    // checked against the wrapper's node base.
    vendor_ =
        std::static_pointer_cast<rclcpp::Node>(driver_->get_node_instance());
    if (vendor_->get_node_base_interface() !=
        driver_->get_node_base_interface()) {
      vendor_.reset();
      return absl::InternalError(
          absl::StrCat(kDriverClass, " is not an rclcpp::Node"));
    }

    // Flowstate's settings are parameter calls on this node: serve the
    // vendor's.
    parameter_service_ = std::make_shared<rclcpp::ParameterService>(
        get_node_base_interface(), get_node_services_interface(),
        vendor_->get_node_parameters_interface().get());
    settings_changed_handle_ = vendor_->add_post_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) {
          const bool restreams = absl::c_any_of(
              parameters, [](const rclcpp::Parameter& parameter) {
                return absl::EndsWith(parameter.get_name(), "_profile") ||
                       absl::EndsWith(parameter.get_name(), "_format");
              });
          const rclcpp::Time now = get_clock()->now();
          absl::MutexLock lock(&mutex_);
          t_settings_changed_ = now;
          restart_pending_ = restart_pending_ || restreams;
        });

    driver_executor_ =
        std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    driver_executor_->add_node(driver_->get_node_base_interface());
    // Last, so any exception above leaves no thread running.
    driver_thread_ = std::thread([this]() {
      try {
        driver_executor_->spin();
      } catch (const std::exception& e) {
        if (rclcpp::ok()) {
          RCLCPP_ERROR(get_logger(), "realsense2_camera executor failed: %s",
                       e.what());
        }
      }
    });
  } catch (const std::exception& e) {
    return absl::InternalError(
        absl::StrCat("starting ", kDriverClass, ": ", e.what()));
  }

  RCLCPP_INFO(get_logger(), "realsense2_camera running as %s",
              absl::StrCat(kDriverNamespace, "/", CameraName()).c_str());
  return absl::OkStatus();
}

void AdapterNode::StopDriver() {
  // The proxy and the callback reach into the vendor node: they go first.
  parameter_service_.reset();
  settings_changed_handle_.reset();
  vendor_.reset();
  if (driver_executor_) driver_executor_->cancel();
  if (driver_thread_.joinable()) driver_thread_.join();
  // The executor's entity collection holds weak references to the vendor
  // node's subscriptions and timers, whose control blocks the vendor library's
  // code made: released after the library is unloaded, they call into its
  // unmapped vtables (the container's segfault on every teardown).
  driver_executor_.reset();
  driver_.reset();
  driver_factory_.reset();
  driver_loader_.reset();
}

void AdapterNode::Subscribe(size_t stream) {
  rclcpp::SubscriptionOptions options;
  options.callback_group = subscription_cb_group_;
  const rclcpp::QoS qos = rclcpp::SensorDataQoS();
  subscriptions_.push_back(create_subscription<CameraInfo>(
      InfoTopic(stream), qos,
      [this, stream](CameraInfo::UniquePtr msg) {
        const rclcpp::Time now = get_clock()->now();
        std::shared_ptr<const CameraInfo> info(std::move(msg));
        absl::MutexLock lock(&mutex_);
        streams_[stream].info = std::move(info);
        streams_[stream].t_last_info = now;
      },
      options));
  subscriptions_.push_back(create_subscription<Image>(
      ImageTopic(stream), qos,
      [this, stream](Image::UniquePtr msg) {
        const rclcpp::Time now = get_clock()->now();
        std::shared_ptr<const Image> image(std::move(msg));
        absl::MutexLock lock(&mutex_);
        StreamState& state = streams_[stream];
        state.image = std::move(image);
        ++state.frame_count;
        state.t_last_frame = now;
      },
      options));
}

bool AdapterNode::DeviceOpened() const {
  return vendor_ && vendor_->has_parameter(std::string(kAlignDepthParameter));
}

std::array<bool, AdapterNode::kStreamCount> AdapterNode::ServedStreams() const {
  if (!DeviceOpened()) return {};
  const auto on = [this](absl::string_view name) {
    rclcpp::Parameter parameter;
    return vendor_->get_parameter(std::string(name), parameter) &&
           parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL &&
           parameter.as_bool();
  };
  std::array<bool, kStreamCount> served;
  for (size_t i = 0; i < kStreamCount; ++i) {
    served[i] = on(kStreams[i].enable_parameter);
  }
  served[kDepth] = served[kDepth] && served[kColor] && on(kAlignDepthParameter);
  return served;
}

std::array<std::optional<ImageDimensions>, AdapterNode::kStreamCount>
AdapterNode::ExpectedStreamSizes() const {
  std::array<std::optional<ImageDimensions>, kStreamCount> expected;
  if (!vendor_) return expected;
  const auto read_profile =
      [this](absl::string_view name) -> std::optional<ImageDimensions> {
    rclcpp::Parameter parameter;
    if (!vendor_->get_parameter(std::string(name), parameter) ||
        parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
      return std::nullopt;
    }
    return ParseProfileDimensions(parameter.as_string());
  };
  // D455/D435/D415 expose color on rgb_camera; D405 serves color from
  // depth_module. Aligned depth always matches color's resolution.
  std::optional<ImageDimensions> color_size =
      read_profile(kRgbColorProfileParam);
  if (!color_size) {
    color_size = read_profile(kDepthColorProfileParam);
  }
  expected[kColor] = color_size;
  expected[kDepth] = color_size;
  const std::optional<ImageDimensions> infra_size =
      read_profile(kInfraProfileParam);
  expected[kInfra1] = infra_size;
  expected[kInfra2] = infra_size;
  return expected;
}

void AdapterNode::RestartStreamsIfPending() {
  absl::MutexLock restart_lock(&restart_mutex_);
  if (!DeviceOpened()) return;
  while (!stop_requested_) {
    {
      absl::MutexLock lock(&mutex_);
      if (!restart_pending_) return;
      restart_pending_ = false;
    }

    // Re-setting enable_color to its current value triggers realsense2_camera's
    // asynchronous _monitoring_pc thread to run updateSensors() across every
    // sensor whose wanted profiles differ from its active profiles.
    try {
      rclcpp::Parameter enable_param;
      if (vendor_->get_parameter(std::string(kStreams[kColor].enable_parameter),
                                 enable_param)) {
        const rcl_interfaces::msg::SetParametersResult result =
            vendor_->set_parameter(enable_param);
        if (!result.successful) {
          RCLCPP_ERROR(get_logger(), "restarting the streams failed: %s",
                       result.reason.c_str());
        }
      }
    } catch (const std::exception& e) {
      if (rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "restarting the streams threw: %s",
                     e.what());
      }
    }

    const rclcpp::Time now = get_clock()->now();
    absl::MutexLock lock(&mutex_);
    t_settings_changed_ = now;
  }
}

std::string AdapterNode::CameraName() const {
  return absl::StrCat("camera_", serial_);
}

std::string AdapterNode::ColorImageTopic() const { return ImageTopic(kColor); }

std::string AdapterNode::ImageTopic(size_t stream) const {
  return absl::StrFormat("%s/%s/%s", kDriverNamespace, CameraName(),
                         kStreams[stream].image_topic);
}

std::string AdapterNode::InfoTopic(size_t stream) const {
  return absl::StrFormat("%s/%s/%s", kDriverNamespace, CameraName(),
                         kStreams[stream].info_topic);
}

void AdapterNode::CacheExtrinsics() {
  if (!tf_buffer_) return;
  // The camera frame is the color optical frame and each sensor's the one its
  // images are stamped in. Color and aligned depth are the camera frame itself;
  // infrared frames still to place are read under the lock, looked up outside.
  std::string camera_frame;
  std::array<std::string, kStreamCount> frames;
  {
    absl::MutexLock lock(&mutex_);
    const StreamState& color = streams_[kColor];
    if (!color.image) return;
    camera_frame = color.image->header.frame_id;
    for (size_t i = 0; i < kStreamCount; ++i) {
      StreamState& stream = streams_[i];
      if (stream.camera_t_sensor || !stream.image) continue;
      const std::string& frame = stream.image->header.frame_id;
      if (i == kColor || i == kDepth || frame == camera_frame) {
        stream.camera_t_sensor = MakeIdentityTransform(camera_frame, frame);
      } else {
        frames[i] = frame;
      }
    }
  }
  std::array<std::optional<geometry_msgs::msg::TransformStamped>, kStreamCount>
      found;
  for (size_t i = 0; i < kStreamCount; ++i) {
    if (frames[i].empty()) continue;
    try {
      found[i] = tf_buffer_->lookupTransform(camera_frame, frames[i],
                                             tf2::TimePointZero);
    } catch (const tf2::TransformException& e) {
      RCLCPP_DEBUG(get_logger(), "%.*s extrinsics not yet known: %s",
                   static_cast<int>(kStreams[i].sensor_name.size()),
                   kStreams[i].sensor_name.data(), e.what());
    }
  }
  bool all_known = true;
  {
    absl::MutexLock lock(&mutex_);
    for (size_t i = 0; i < kStreamCount; ++i) {
      StreamState& stream = streams_[i];
      if (found[i]) {
        const geometry_msgs::msg::Transform& transform = found[i]->transform;
        RCLCPP_INFO(
            get_logger(),
            "%.*s extrinsics: [%.4f, %.4f, %.4f], [%.4f, %.4f, %.4f, %.4f]",
            static_cast<int>(kStreams[i].sensor_name.size()),
            kStreams[i].sensor_name.data(), transform.translation.x,
            transform.translation.y, transform.translation.z,
            transform.rotation.x, transform.rotation.y, transform.rotation.z,
            transform.rotation.w);
        stream.camera_t_sensor = std::move(found[i]);
      }
      all_known = all_known && stream.camera_t_sensor.has_value();
    }
  }
  // Static for the device: the listener's thread is no longer needed.
  if (all_known) {
    tf_listener_.reset();
    tf_buffer_.reset();
  }
}

void AdapterNode::CheckLiveness() {
  const bool opened = DeviceOpened();
  const std::array<bool, kStreamCount> served =
      opened ? ServedStreams() : std::array<bool, kStreamCount>{};
  const rclcpp::Time now = get_clock()->now();
  absl::Duration silent = absl::ZeroDuration();
  std::string silence;
  {
    absl::MutexLock lock(&mutex_);
    if (!opened) {
      silent = absl::Nanoseconds((now - t_started_).nanoseconds());
      silence = "no device opened by the driver";
    } else {
      for (size_t i = 0; i < kStreamCount; ++i) {
        StreamState& stream = streams_[i];
        if (served[i] && !stream.served) stream.t_last_frame = now;
        stream.served = served[i];
        if (!served[i]) continue;
        const absl::Duration elapsed =
            absl::Nanoseconds((now - stream.t_last_frame).nanoseconds());
        if (elapsed > silent) {
          silent = elapsed;
          silence = absl::StrCat("no ", kStreams[i].sensor_name, " frame");
        }
      }
    }
  }
  if (silent > kLivenessTimeout) {
    RCLCPP_ERROR(get_logger(),
                 "%s for %s: tearing the node down, the spawner re-creates it",
                 silence.c_str(), absl::FormatDuration(silent).c_str());
    {
      absl::MutexLock lock(&mutex_);
      stop_requested_ = true;
    }
    executor_.cancel();
  }
}

absl::Status AdapterNode::Main() {
  if (stop_requested_) {
    return absl::OkStatus();
  }
  if (const absl::Status status = StartDriver(); !status.ok()) {
    StopDriver();
    return status;
  }
  if (stop_requested_) {
    StopDriver();
    return absl::OkStatus();
  }

  absl::Status status = absl::OkStatus();
  try {
    liveness_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      if (stop_requested_) {
        executor_.cancel();
        return;
      }
      CacheExtrinsics();
      CheckLiveness();
    });
    executor_.add_node(get_node_base_interface());
    if (!stop_requested_) {
      executor_.spin();
    }
  } catch (const std::exception& e) {
    if (rclcpp::ok()) {
      status = absl::InternalError(absl::StrCat("Executor failed: ", e.what()));
    }
  }
  {
    absl::MutexLock lock(&mutex_);
    stop_requested_ = true;
  }
  liveness_timer_.reset();
  StopDriver();
  RCLCPP_INFO(get_logger(), "exiting AdapterNode::Main()");
  return status;
}

absl::StatusOr<snapshot_interfaces::srv::Describe::Response>
AdapterNode::BuildDescribeResponse() {
  if (stop_requested_) {
    return absl::UnavailableError("Adapter stopping");
  }
  if (!DeviceOpened()) {
    return absl::UnavailableError("the driver has not opened the device yet");
  }
  snapshot_interfaces::srv::Describe::Response response;
  absl::MutexLock lock(&mutex_);
  if (stop_requested_) {
    return absl::UnavailableError("Adapter stopping");
  }
  std::vector<absl::string_view> pending;
  for (size_t i = 0; i < kStreamCount; ++i) {
    if (!streams_[i].info || !streams_[i].camera_t_sensor) {
      pending.push_back(kStreams[i].sensor_name);
    }
  }
  if (!pending.empty()) {
    return absl::UnavailableError(
        absl::StrCat("CameraInfo or extrinsics not yet received: ",
                     absl::StrJoin(pending, ", ")));
  }
  for (size_t i = 0; i < kStreamCount; ++i) {
    SensorInfo sensor = BuildSensorInformation(
        *streams_[i].info, std::string(kStreams[i].sensor_name), ImageTopic(i),
        *streams_[i].camera_t_sensor);
    sensor.sensor_type = kStreams[i].sensor_type;
    response.sensors.push_back(std::move(sensor));
  }
  return response;
}

absl::StatusOr<snapshot_interfaces::srv::Snapshot::Response>
AdapterNode::BuildSnapshotResponse() {
  if (stop_requested_) {
    return absl::UnavailableError("Adapter stopping");
  }
  if (!DeviceOpened()) {
    return absl::UnavailableError("the driver has not opened the device yet");
  }
  // A profile set by this capture's settings: trigger the vendor's async
  // stream restart now, after the full batch of camera_settings has been set.
  RestartStreamsIfPending();
  if (stop_requested_) {
    return absl::UnavailableError("Adapter stopping");
  }
  const std::array<bool, kStreamCount> served = ServedStreams();
  if (absl::c_none_of(served, [](bool on) { return on; })) {
    return absl::FailedPreconditionError(
        "no stream is on: every enable_* parameter is false");
  }
  const std::array<std::optional<ImageDimensions>, kStreamCount>
      expected_sizes = ExpectedStreamSizes();

  // Grab immutable shared_ptrs under mutex_ and copy/convert payloads outside
  // it so 30 Hz subscription callbacks are never stalled by snapshot work.
  std::array<std::shared_ptr<const CameraInfo>, kStreamCount> infos;
  std::array<std::shared_ptr<const Image>, kStreamCount> images;
  {
    absl::MutexLock lock(&mutex_);
    std::array<uint64_t, kStreamCount> targets;
    for (size_t i = 0; i < kStreamCount; ++i) {
      targets[i] = streams_[i].frame_count;
    }
    // A frame and matching CameraInfo of each served stream newer than the
    // request, received after the last settings change, and matching the active
    // *_profile dimensions (gating out pre-restart frames during the vendor's
    // asynchronous _monitoring_pc restart window).
    const auto fresh = [this, &targets, &expected_sizes](
                           size_t i) ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
      const StreamState& stream = streams_[i];
      if (!stream.info || !stream.image || stream.frame_count <= targets[i] ||
          stream.t_last_frame < t_settings_changed_ ||
          stream.t_last_info < t_settings_changed_ ||
          stream.info->width != stream.image->width ||
          stream.info->height != stream.image->height) {
        return false;
      }
      if (expected_sizes[i] &&
          (stream.image->width != expected_sizes[i]->width ||
           stream.image->height != expected_sizes[i]->height)) {
        return false;
      }
      return true;
    };
    const auto all_fresh = [this, &served,
                            &fresh]() ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
      if (stop_requested_) return true;
      for (size_t i = 0; i < kStreamCount; ++i) {
        if (served[i] && !fresh(i)) return false;
      }
      if (served[kDepth] && served[kColor]) {
        if (streams_[kDepth].image->width != streams_[kColor].image->width ||
            streams_[kDepth].image->height != streams_[kColor].image->height) {
          return false;
        }
      }
      return true;
    };
    const bool ready =
        mutex_.AwaitWithTimeout(absl::Condition(&all_fresh), kImageDeadline);
    if (stop_requested_) {
      return absl::UnavailableError("Adapter stopping");
    }
    if (!ready) {
      std::vector<absl::string_view> stale;
      for (size_t i = 0; i < kStreamCount; ++i) {
        if (served[i] && !fresh(i)) stale.push_back(kStreams[i].sensor_name);
      }
      if (stale.empty()) stale.push_back("aligned depth");
      return absl::DeadlineExceededError(absl::StrFormat(
          "no new %s frames within %s", absl::StrJoin(stale, ", "),
          absl::FormatDuration(kImageDeadline)));
    }

    for (size_t i = 0; i < kStreamCount; ++i) {
      if (!served[i]) continue;
      infos[i] = streams_[i].info;
      images[i] = streams_[i].image;
    }
  }

  snapshot_interfaces::srv::Snapshot::Response response;
  for (size_t i = 0; i < kStreamCount; ++i) {
    if (!images[i]) continue;
    ImageSnapshot snapshot;
    snapshot.topic_name = ImageTopic(i);
    snapshot.camera_info = *infos[i];
    if (i == kDepth) {
      absl::StatusOr<Image> depth = ConvertDepthToMeters(*images[i]);
      if (!depth.ok()) return depth.status();
      snapshot.image = *std::move(depth);
    } else {
      snapshot.image = *images[i];
    }
    response.images.push_back(std::move(snapshot));
  }
  return response;
}

}  // namespace flowstate_realsense
