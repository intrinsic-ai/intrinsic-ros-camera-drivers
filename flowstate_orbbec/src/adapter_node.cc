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

#include "flowstate_orbbec/adapter_node.h"

#include <memory>

#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "flowstate_orbbec/spawner_node.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "snapshot_interfaces/msg/image_snapshot.hpp"
#include "std_srvs/srv/empty.hpp"

namespace flowstate_orbbec {

constexpr std::string_view kOrbbecNodeName = "orbbec_camera_node";
constexpr std::string_view kOrbbecNodeNamespacePrefix = "orbbec/camera_";
constexpr absl::Duration kImageDeadline = absl::Seconds(1);

AdapterNode::AdapterNode(const std::string& serial,
                         const std::vector<std::string>& locators)
    : flowstate_common::BaseAdapterNode(
          serial, locators, "orbbec",
          rclcpp::NodeOptions().use_intra_process_comms(true)) {
  InitializeParameters();
  subscription_cb_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  software_trigger_client_ = create_client<std_srvs::srv::SetBool>(
      absl::StrFormat("/orbbec/camera_%s/send_software_trigger", serial_));

  CreateOrbbecNode();

  // Create a TF Listener, which will be used to query extrinsics
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = subscription_cb_group_;

  // Subscribe to camera infos
  color_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      absl::StrFormat("orbbec/camera_%s/color/camera_info", serial_), 2,
      [this](sensor_msgs::msg::CameraInfo::UniquePtr msg) {
        absl::MutexLock lock(&this->mutex_);
        this->color_camera_info_ = std::move(msg);
        this->color_info_sub_.reset();
      }, sub_options);

  left_ir_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      absl::StrFormat("orbbec/camera_%s/left_ir/camera_info", serial_), 2,
      [this](sensor_msgs::msg::CameraInfo::UniquePtr msg) {
        absl::MutexLock lock(&this->mutex_);
        this->left_ir_camera_info_ = std::move(msg);
        this->left_ir_info_sub_.reset();
      }, sub_options);

  right_ir_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      absl::StrFormat("orbbec/camera_%s/right_ir/camera_info", serial_), 2,
      [this](sensor_msgs::msg::CameraInfo::UniquePtr msg) {
        absl::MutexLock lock(&this->mutex_);
        this->right_ir_camera_info_ = std::move(msg);
        this->right_ir_info_sub_.reset();
      }, sub_options);

  depth_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      absl::StrFormat("orbbec/camera_%s/depth/camera_info", serial_), 2,
      [this](sensor_msgs::msg::CameraInfo::UniquePtr msg) {
        absl::MutexLock lock(&this->mutex_);
        this->depth_camera_info_ = std::move(msg);
        this->depth_info_sub_.reset();
      }, sub_options);

  // Subscribe to color image
  color_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      ColorImageTopic(), 5,
      [this](sensor_msgs::msg::Image::UniquePtr msg) {
        {
          absl::MutexLock timeout_lock(&this->timeout_mutex_);
          this->t_last_color_image_ = this->get_clock()->now();
        }
        absl::MutexLock lock(&this->mutex_);
        this->color_image_ = std::move(msg);
        this->color_frame_count_++;
      },
      sub_options);

  // Subscribe to IR images
  left_ir_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      LeftIrImageTopic(), 5,
      [this](sensor_msgs::msg::Image::UniquePtr msg) {
        {
          absl::MutexLock timeout_lock(&this->timeout_mutex_);
          this->t_last_left_ir_image_ = this->get_clock()->now();
        }
        absl::MutexLock lock(&this->mutex_);
        this->left_ir_image_ = std::move(msg);
        this->left_ir_frame_count_++;
      },
      sub_options);

  right_ir_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      RightIrImageTopic(), 5, [this](sensor_msgs::msg::Image::UniquePtr msg) {
        {
          absl::MutexLock timeout_lock(&this->timeout_mutex_);
          this->t_last_right_ir_image_ = this->get_clock()->now();
        }
        absl::MutexLock lock(&this->mutex_);
        this->right_ir_image_ = std::move(msg);
        this->right_ir_frame_count_++;
      }, sub_options);

  depth_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      DepthImageTopic(), 5, [this](sensor_msgs::msg::Image::UniquePtr msg) {
        {
          absl::MutexLock timeout_lock(&this->timeout_mutex_);
          this->t_last_depth_image_ = this->get_clock()->now();
        }
        absl::MutexLock lock(&this->mutex_);
        this->depth_image_ = std::move(msg);
        this->depth_frame_count_++;
      }, sub_options);

  // Create Flowstate services
  CreateFlowstateServices();

  // Start background executor thread
  StartExecutorThread();
}

AdapterNode::~AdapterNode() {
  RCLCPP_INFO(get_logger(), "AdapterNode dtor");
}

std::string AdapterNode::ColorImageTopic() const {
  return absl::StrFormat("/orbbec/camera_%s/color/image_raw", serial_);
}

std::string AdapterNode::LeftIrImageTopic() const {
  return absl::StrFormat("/orbbec/camera_%s/left_ir/image_raw", serial_);
}

std::string AdapterNode::RightIrImageTopic() const {
  return absl::StrFormat("/orbbec/camera_%s/right_ir/image_raw", serial_);
}

std::string AdapterNode::DepthImageTopic() const {
  return absl::StrFormat("/orbbec/camera_%s/depth/image_raw", serial_);
}

void AdapterNode::InitializeParameters() {
  set_auto_exposure_client_ = create_client<std_srvs::srv::SetBool>(
      absl::StrFormat("/orbbec/camera_%s/set_color_auto_exposure", serial_));
  set_exposure_client_ = create_client<orbbec_camera_msgs::srv::SetInt32>(
      absl::StrFormat("/orbbec/camera_%s/set_color_exposure", serial_));

  set_auto_white_balance_client_ = create_client<std_srvs::srv::SetBool>(
      absl::StrFormat("/orbbec/camera_%s/set_auto_white_balance", serial_));
  set_white_balance_client_ = create_client<orbbec_camera_msgs::srv::SetInt32>(
      absl::StrFormat("/orbbec/camera_%s/set_white_balance", serial_));
  set_laser_enable_client_ = create_client<std_srvs::srv::SetBool>(
      absl::StrFormat("/orbbec/camera_%s/set_laser_enable", serial_));

  set_gain_client_ = create_client<orbbec_camera_msgs::srv::SetInt32>(
      absl::StrFormat("/orbbec/camera_%s/set_color_gain", serial_));

  toggle_color_client_ = create_client<std_srvs::srv::SetBool>(
      absl::StrFormat("/orbbec/camera_%s/toggle_color", serial_));
  toggle_depth_client_ = create_client<std_srvs::srv::SetBool>(
      absl::StrFormat("/orbbec/camera_%s/toggle_depth", serial_));
  toggle_left_ir_client_ = create_client<std_srvs::srv::SetBool>(
      absl::StrFormat("/orbbec/camera_%s/toggle_left_ir", serial_));
  toggle_right_ir_client_ = create_client<std_srvs::srv::SetBool>(
      absl::StrFormat("/orbbec/camera_%s/toggle_right_ir", serial_));

  pre_set_parameters_callback_handle_ =
      add_pre_set_parameters_callback(std::bind(
          &AdapterNode::PreSetParametersCallback, this, std::placeholders::_1));
  on_set_parameters_callback_handle_ = add_on_set_parameters_callback(std::bind(
      &AdapterNode::SetParametersCallback, this, std::placeholders::_1));
  post_set_parameters_callback_handle_ = add_post_set_parameters_callback(
      std::bind(&AdapterNode::PostSetParametersCallback, this,
                std::placeholders::_1));

  rcl_interfaces::msg::FloatingPointRange fps_range;
  fps_range.from_value = 5.0;
  fps_range.to_value = 30.0;
  fps_range.step = 5.0;
  rcl_interfaces::msg::ParameterDescriptor fps_descriptor;
  fps_descriptor.name = "fps";
  fps_descriptor.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
  fps_descriptor.description = "FPS (framerate)";
  fps_descriptor.read_only = false;
  fps_descriptor.floating_point_range.push_back(fps_range);
  declare_parameter("fps", 10.0, fps_descriptor);

  rcl_interfaces::msg::ParameterDescriptor enable_rgb_descriptor;
  enable_rgb_descriptor.name = "enable_rgb";
  enable_rgb_descriptor.type = rclcpp::ParameterType::PARAMETER_BOOL;
  enable_rgb_descriptor.description = "Stream the RGB sensor.";
  enable_rgb_descriptor.read_only = false;
  declare_parameter("enable_rgb", true, enable_rgb_descriptor);

  rcl_interfaces::msg::ParameterDescriptor enable_left_ir_descriptor;
  enable_left_ir_descriptor.name = "enable_left_ir";
  enable_left_ir_descriptor.type = rclcpp::ParameterType::PARAMETER_BOOL;
  enable_left_ir_descriptor.description = "Stream the left IR sensor.";
  enable_left_ir_descriptor.read_only = false;
  declare_parameter("enable_left_ir", true, enable_left_ir_descriptor);

  rcl_interfaces::msg::ParameterDescriptor enable_right_ir_descriptor;
  enable_right_ir_descriptor.name = "enable_right_ir";
  enable_right_ir_descriptor.type = rclcpp::ParameterType::PARAMETER_BOOL;
  enable_right_ir_descriptor.description = "Stream the right IR sensor.";
  enable_right_ir_descriptor.read_only = false;
  declare_parameter("enable_right_ir", true, enable_right_ir_descriptor);

  rcl_interfaces::msg::ParameterDescriptor enable_depth_descriptor;
  enable_depth_descriptor.name = "enable_depth";
  enable_depth_descriptor.type = rclcpp::ParameterType::PARAMETER_BOOL;
  enable_depth_descriptor.description = "Stream the depth sensor.";
  enable_depth_descriptor.read_only = false;
  declare_parameter("enable_depth", true, enable_depth_descriptor);

  rcl_interfaces::msg::ParameterDescriptor auto_exposure_descriptor;
  auto_exposure_descriptor.name = "auto_exposure";
  auto_exposure_descriptor.type = rclcpp::ParameterType::PARAMETER_BOOL;
  auto_exposure_descriptor.description = "Toggle auto_exposure.";
  auto_exposure_descriptor.read_only = false;
  declare_parameter("auto_exposure", true, auto_exposure_descriptor);

  rcl_interfaces::msg::ParameterDescriptor enable_laser_descriptor;
  enable_laser_descriptor.name = "enable_laser";
  enable_laser_descriptor.type = rclcpp::ParameterType::PARAMETER_BOOL;
  enable_laser_descriptor.description = "Toggle laser dot projector.";
  enable_laser_descriptor.read_only = false;
  declare_parameter("enable_laser", true, enable_laser_descriptor);

  rcl_interfaces::msg::FloatingPointRange exposure_range;
  exposure_range.from_value = 0.0001;
  exposure_range.to_value = 0.1;
  exposure_range.step = 0.0001;
  rcl_interfaces::msg::ParameterDescriptor exposure_descriptor;
  exposure_descriptor.name = "exposure";
  exposure_descriptor.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
  exposure_descriptor.description = "Exposure time in seconds.";
  exposure_descriptor.read_only = false;
  exposure_descriptor.floating_point_range.push_back(exposure_range);
  declare_parameter("exposure", 0.01, exposure_descriptor);

  rcl_interfaces::msg::ParameterDescriptor auto_white_balance_descriptor;
  auto_white_balance_descriptor.name = "auto_white_balance";
  auto_white_balance_descriptor.type = rclcpp::ParameterType::PARAMETER_BOOL;
  auto_white_balance_descriptor.description = "Toggle auto_white_balance.";
  auto_white_balance_descriptor.read_only = false;
  declare_parameter("auto_white_balance", true, auto_white_balance_descriptor);

  rcl_interfaces::msg::IntegerRange white_balance_range;
  white_balance_range.from_value = 2800;
  white_balance_range.to_value = 6500;
  white_balance_range.step = 1;
  rcl_interfaces::msg::ParameterDescriptor white_balance_descriptor;
  white_balance_descriptor.name = "white_balance";
  white_balance_descriptor.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  white_balance_descriptor.description = "White balance in K.";
  white_balance_descriptor.read_only = false;
  white_balance_descriptor.integer_range.push_back(white_balance_range);
  declare_parameter("white_balance", 4000, white_balance_descriptor);

  rcl_interfaces::msg::IntegerRange gain_range;
  gain_range.from_value = 0;
  gain_range.to_value = 128;
  gain_range.step = 1;
  rcl_interfaces::msg::ParameterDescriptor gain_descriptor;
  gain_descriptor.name = "gain";
  gain_descriptor.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  gain_descriptor.description = "Sensor gain";
  gain_descriptor.read_only = false;
  gain_descriptor.integer_range.push_back(gain_range);
  declare_parameter("gain", 16, gain_descriptor);

  rcl_interfaces::msg::ParameterDescriptor streaming_descriptor;
  streaming_descriptor.name = "streaming";
  streaming_descriptor.type = rclcpp::ParameterType::PARAMETER_BOOL;
  streaming_descriptor.description =
      "Toggle continuous streaming (true) vs snapshot on-demand (false) mode.";
  streaming_descriptor.read_only = false;
  declare_parameter("streaming", false, streaming_descriptor);
  streaming_ = get_parameter("streaming").as_bool();
}

// PreSetParametersCallback is used to add or adjust the parameter vector
void AdapterNode::PreSetParametersCallback(
    std::vector<rclcpp::Parameter>& parameters) {
  const bool sets_exposure =
      std::find_if(parameters.begin(), parameters.end(),
                   [](const rclcpp::Parameter& param) {
                     return param.get_name() == "exposure";
                   }) != parameters.end();
  const bool sets_gain = std::find_if(parameters.begin(), parameters.end(),
                                      [](const rclcpp::Parameter& param) {
                                        return param.get_name() == "gain";
                                      }) != parameters.end();
  const bool sets_auto_exposure =
      std::find_if(parameters.begin(), parameters.end(),
                   [](const rclcpp::Parameter& param) {
                     return param.get_name() == "auto_exposure";
                   }) != parameters.end();
  if ((sets_exposure || sets_gain) && !sets_auto_exposure) {
    parameters.insert(parameters.begin(),
                      rclcpp::Parameter("auto_exposure", false));
  }

  // it seems white balance can't be handled this way; it always
  // resets white balance to a known value whenever it is disabled.
  // This probably needs to be handled by querying if auto_white_balance
  // is set to true, and if it is, set it to false, and start a one-shot
  // timer that will set the target white balance value after 100ms or so.
  const bool sets_white_balance =
      std::find_if(parameters.begin(), parameters.end(),
                   [](const rclcpp::Parameter& param) {
                     return param.get_name() == "white_balance";
                   }) != parameters.end();
  const bool sets_auto_white_balance =
      std::find_if(parameters.begin(), parameters.end(),
                   [](const rclcpp::Parameter& param) {
                     return param.get_name() == "auto_white_balance";
                   }) != parameters.end();
  if (sets_white_balance && !sets_auto_white_balance) {
    parameters.insert(parameters.begin(),
                      rclcpp::Parameter("auto_white_balance", false));
  }
}

rcl_interfaces::msg::SetParametersResult AdapterNode::SetParametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const rclcpp::Parameter& parameter : parameters) {
    if (parameter.get_name() == "exposure") {
      if (parameter.as_double() < 0.0001 || parameter.as_double() > 0.1) {
        result.successful = false;
        result.reason = "Exposure must be between 0.0001 and 0.1";
        break;
      }
    } else if (parameter.get_name() == "white_balance") {
      if (parameter.as_int() < 2800 || parameter.as_int() > 6500) {
        result.successful = false;
        result.reason = "White balance must be between 2800 and 6500";
        break;
      }
    } else if (parameter.get_name() == "gain") {
      if (parameter.as_int() < 0 || parameter.as_int() > 128) {
        result.successful = false;
        result.reason = "Gain must be between 0 and 128";
        break;
      }
    } else if (parameter.get_name() == "fps") {
      if (parameter.as_double() != 5.0 && parameter.as_double() != 10.0 &&
          parameter.as_double() != 15.0 && parameter.as_double() != 30.0) {
        result.successful = false;
        result.reason =
            "fps must be either 5, 10, 15, or 30, and fit in 1 gigabit/sec.";
        break;
      }
    }
  }
  return result;
}

// PostSetParametersCallback() is where we use the validated parameters
// in this case by forwarding them to the Orbbec Driver node.
void AdapterNode::PostSetParametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {

  // First, check if we need to adjust FPS, since that requires a camera reset
  // If we are resetting the camera, many of the subsequent changes are already
  // running, so we need to not toggle them again.
  bool reset_complete_ = false;
  for (const rclcpp::Parameter& parameter : parameters) {
    if (parameter.get_name() == "fps") {
      if (fps_ == parameter.as_double()) {
        RCLCPP_INFO(get_logger(), "Ignoring identical FPS request: %.1f", fps_);
        continue;
      }
      RCLCPP_INFO(get_logger(), "Resetting camera to set fps: %.1f", fps_);
      fps_ = parameter.as_double();
      {
        absl::MutexLock timeout_lock(&timeout_mutex_);
        t_last_color_image_ = get_clock()->now();
        t_last_left_ir_image_ = get_clock()->now();
        t_last_right_ir_image_ = get_clock()->now();
        t_last_depth_image_ = get_clock()->now();
      }

      CreateOrbbecNode();
      reset_complete_ = true;
    }
  }

  // Now, handle everything _other_ than fps
  for (const rclcpp::Parameter& parameter : parameters) {
    std::string value_str = "(type not converted to string)";
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      value_str = parameter.as_bool() ? "true" : "false";
    } else if (parameter.get_type() ==
               rclcpp::ParameterType::PARAMETER_DOUBLE) {
      value_str = absl::StrFormat("%.6f", parameter.as_double());
    } else if (parameter.get_type() ==
               rclcpp::ParameterType::PARAMETER_INTEGER) {
      value_str = absl::StrFormat("%d", parameter.as_int());
    } else if (parameter.get_type() ==
               rclcpp::ParameterType::PARAMETER_STRING) {
      value_str = parameter.as_string();
    }
    RCLCPP_INFO(get_logger(), "PostSetParametersCallback: %s %s",
                parameter.get_name().c_str(), value_str.c_str());

    // Set the parameter by using the relevant service client to
    // send an async request.
    if (parameter.get_name() == "exposure") {
      CallAsyncSet(set_exposure_client_,
                   static_cast<int>(10000.0 * parameter.as_double()));
    } else if (parameter.get_name() == "auto_exposure") {
      CallAsyncSet(set_auto_exposure_client_, parameter.as_bool());
    } else if (parameter.get_name() == "auto_white_balance") {
      // This one is tricky. Only disable it if it's requested to be disabled
      // and it is currently enabled. If it is "re-disabled" while already
      // set to disabled, then it resets the target white balance.
      // But we don't want it to "get stuck", if the driver reboots, so always
      // send requests to enable it.
      if ((parameter.as_bool() != auto_white_balance_) || parameter.as_bool()) {
        CallAsyncSet(set_auto_white_balance_client_, parameter.as_bool(),
                     &auto_white_balance_);
      }
    } else if (parameter.get_name() == "white_balance") {
      CallAsyncSet(set_white_balance_client_, parameter.as_int());
    } else if (parameter.get_name() == "gain") {
      CallAsyncSet(set_gain_client_, parameter.as_int());
    } else if (parameter.get_name() == "enable_rgb" && !reset_complete_) {
      absl::MutexLock timeout_lock(&timeout_mutex_);
      t_last_color_image_ = get_clock()->now();
      CallAsyncSet(toggle_color_client_, parameter.as_bool());
    } else if (parameter.get_name() == "enable_left_ir" && !reset_complete_) {
      absl::MutexLock timeout_lock(&timeout_mutex_);
      t_last_left_ir_image_ = get_clock()->now();
      CallAsyncSet(toggle_left_ir_client_, parameter.as_bool());
    } else if (parameter.get_name() == "enable_right_ir" && !reset_complete_) {
      absl::MutexLock timeout_lock(&timeout_mutex_);
      t_last_right_ir_image_ = get_clock()->now();
      CallAsyncSet(toggle_right_ir_client_, parameter.as_bool());
    } else if (parameter.get_name() == "enable_depth" && !reset_complete_) {
      absl::MutexLock timeout_lock(&timeout_mutex_);
      t_last_depth_image_ = get_clock()->now();
      CallAsyncSet(toggle_depth_client_, parameter.as_bool());
    } else if (parameter.get_name() == "enable_laser") {
      CallAsyncSet(set_laser_enable_client_, parameter.as_bool());
    } else if (parameter.get_name() == "streaming") {
      streaming_ = parameter.as_bool();
      RCLCPP_INFO(get_logger(), "Streaming parameter set to: %s",
                  streaming_ ? "true (pipelined triggering)" : "false (snapshot on-demand)");
      if (streaming_) {
        absl::MutexLock lock(&mutex_);
        if (software_trigger_client_) {
          auto trigger_req = std::make_shared<std_srvs::srv::SetBool::Request>();
          trigger_req->data = true;
          software_trigger_client_->async_send_request(trigger_req);
        }
      }
    }
  }
}

absl::Status AdapterNode::Main() {
  RCLCPP_INFO(get_logger(), "AdapterNode::Main()");
  {
    absl::MutexLock timeout_lock(&timeout_mutex_);
    t_last_color_image_ = get_clock()->now();
    t_last_left_ir_image_ = get_clock()->now();
    t_last_right_ir_image_ = get_clock()->now();
    t_last_depth_image_ = get_clock()->now();
  }
  // use 4 threads just to ensure we keep some free
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  liveness_timer_ =
      create_wall_timer(std::chrono::seconds(1), [this, &executor]() {
        bool should_reboot = false;
        {
          absl::MutexLock timeout_lock(&timeout_mutex_);
          const rclcpp::Time t = get_clock()->now();

          if (streaming_) {
            if (IsRgbEnabled() && (t - t_last_color_image_).seconds() > 30.0) {
              RCLCPP_ERROR(get_logger(),
                           "No new color image arrived for 30 seconds");
              should_reboot = true;
            }
            if (IsLeftIrEnabled() &&
                (t - t_last_left_ir_image_).seconds() > 30.0) {
              RCLCPP_ERROR(get_logger(),
                           "No new left IR image arrived for 30 seconds");
              should_reboot = true;
            }
            if (IsRightIrEnabled() &&
                (t - t_last_right_ir_image_).seconds() > 30.0) {
              RCLCPP_ERROR(get_logger(),
                           "No new right IR image arrived for 30 seconds");
              should_reboot = true;
            }
            if (IsDepthEnabled() && (t - t_last_depth_image_).seconds() > 30.0) {
              RCLCPP_ERROR(get_logger(),
                           "No new depth image arrived for 30 seconds");
              should_reboot = true;
            }
          }
        }

        if (should_reboot) {
          {
            absl::MutexLock timeout_lock(&timeout_mutex_);
            t_last_color_image_ = get_clock()->now();
            t_last_left_ir_image_ = get_clock()->now();
            t_last_right_ir_image_ = get_clock()->now();
            t_last_depth_image_ = get_clock()->now();
          }
          if (reboot_count_++ < 1) {
            // Try to reboot the device
            absl::MutexLock lock(&SpawnerNode::s_discovery_mutex);
            rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reboot_client =
                create_client<std_srvs::srv::Empty>(absl::StrFormat(
                    "/orbbec/camera_%s/reboot_device", serial_));
            reboot_client->async_send_request(
                std::make_shared<std_srvs::srv::Empty::Request>(),
                [this](
                    rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
                  if (future.valid()) {
                    auto response = future.get();
                    RCLCPP_INFO(this->get_logger(),
                                "Received valid future from reboot client");
                  } else {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Did not receive a valid future from reboot client");
                  }
                });
          } else {
            // Tear down this node. The spawner will re-spawn it soon.
            executor.cancel();
          }
        }
      });

  executor.add_node(this->get_node_base_interface());
  executor.spin();
  RCLCPP_INFO(get_logger(), "Exiting AdapterNode::Main()");
  return absl::OkStatus();
}

absl::StatusOr<snapshot_interfaces::srv::Describe::Response>
AdapterNode::BuildDescribeResponse() {
  snapshot_interfaces::srv::Describe::Response response;
  absl::MutexLock lock(&mutex_);
  absl::Status populate_status = PopulateExtrinsicsIfNeeded();
  if (!populate_status.ok()) {
    return populate_status;
  }

  if (IsRgbEnabled()) {
    if (!color_camera_info_) {
      return absl::UnavailableError(
          "CameraInfo not yet received (waiting for RGB)");
    }
    response.sensors.push_back(BuildSensorInformation(
        *color_camera_info_, "rgb", ColorImageTopic(), *color_transform_));
  }

  if (IsLeftIrEnabled()) {
    if (!left_ir_camera_info_) {
      return absl::UnavailableError(
          "CameraInfo not yet received (waiting for left IR)");
    }
    response.sensors.push_back(
        BuildSensorInformation(*left_ir_camera_info_, "ir_left",
                               LeftIrImageTopic(), *left_ir_transform_));
  }

  if (IsRightIrEnabled()) {
    if (!right_ir_camera_info_) {
      return absl::UnavailableError(
          "CameraInfo not yet received (waiting for right IR)");
    }
    response.sensors.push_back(
        BuildSensorInformation(*right_ir_camera_info_, "ir_right",
                               RightIrImageTopic(), *right_ir_transform_));
  }

  if (IsDepthEnabled()) {
    if (!color_camera_info_) {
      return absl::UnavailableError(
          "CameraInfo not yet received (waiting for color for depth)");
    }
    response.sensors.push_back(BuildSensorInformation(
        *color_camera_info_, "depth", DepthImageTopic(), *color_transform_));
  }

  return response;
}

absl::StatusOr<snapshot_interfaces::srv::Snapshot::Response>
AdapterNode::BuildSnapshotResponse() {
  snapshot_interfaces::srv::Snapshot::Response response;

  if (!streaming_) {
    uint64_t target_color_count = 0;
    uint64_t target_left_ir_count = 0;
    uint64_t target_right_ir_count = 0;
    uint64_t target_depth_count = 0;
    {
      absl::MutexLock lock(&mutex_);
      target_color_count = color_frame_count_;
      target_left_ir_count = left_ir_frame_count_;
      target_right_ir_count = right_ir_frame_count_;
      target_depth_count = depth_frame_count_;
    }

    if (software_trigger_client_) {
      auto trigger_req = std::make_shared<std_srvs::srv::SetBool::Request>();
      trigger_req->data = true;
      software_trigger_client_->async_send_request(trigger_req);
      RCLCPP_INFO(get_logger(), "Sent trigger request async");
    }

    {
      absl::MutexLock lock(&mutex_);
      auto all_new_frames_arrived = [this, target_color_count,
                                     target_left_ir_count,
                                     target_right_ir_count,
                                     target_depth_count]() {
        if (this->IsRgbEnabled() &&
            (!this->color_camera_info_ ||
             this->color_frame_count_ == target_color_count)) {
          // RCLCPP_INFO(get_logger(), "no rgb image yet");
          return false;
        }
        if (this->IsLeftIrEnabled() &&
            (!this->left_ir_camera_info_ ||
             this->left_ir_frame_count_ == target_left_ir_count)) {
          // RCLCPP_INFO(get_logger(), "no left IR image yet");
          return false;
        }
        if (this->IsRightIrEnabled() &&
            (!this->right_ir_camera_info_ ||
             this->right_ir_frame_count_ == target_right_ir_count)) {
          // RCLCPP_INFO(get_logger(), "no right IR image yet");
          return false;
        }

        // The depth image is registered to the color image, so it needs the
        // color_camera_info here.
        if (this->IsDepthEnabled() &&
            (!this->color_camera_info_ ||
             this->depth_frame_count_ == target_depth_count)) {
          // RCLCPP_INFO(get_logger(), "no depth image yet");
          return false;
        }
        return true;
      };
      if (!mutex_.AwaitWithTimeout(
              absl::Condition(&all_new_frames_arrived), kImageDeadline)) {
        return absl::DeadlineExceededError(
            absl::StrFormat("Image(s) did not arrive within %s",
                            absl::FormatDuration(kImageDeadline)));
      }
    }
  }
  RCLCPP_INFO(get_logger(), "got all frames");

  // In streaming mode, proactively trigger the NEXT frame right after
  // AwaitWithTimeout() unblocks so the camera exposes the next frame in the background.
  if (streaming_ && software_trigger_client_) {
    auto next_trigger_req = std::make_shared<std_srvs::srv::SetBool::Request>();
    next_trigger_req->data = true;
    software_trigger_client_->async_send_request(next_trigger_req);
  }

  // Lock and copy the most recent CameraInfo and Image messages
  {
    absl::MutexLock lock(&mutex_);

    if (IsRgbEnabled()) {
      if (!color_camera_info_) {
        return absl::UnavailableError("Color CameraInfo not yet received");
      }
      if (!color_image_) {
        return absl::UnavailableError("Color image not yet received");
      }
      snapshot_interfaces::msg::ImageSnapshot color_snapshot;
      color_snapshot.topic_name = ColorImageTopic();
      color_snapshot.camera_info = *color_camera_info_;
      color_snapshot.image = *color_image_;
      response.images.push_back(std::move(color_snapshot));
    }

    if (IsLeftIrEnabled()) {
      if (!left_ir_camera_info_) {
        return absl::UnavailableError("Left IR CameraInfo not yet received");
      }
      if (!left_ir_image_) {
        return absl::UnavailableError("Left IR Image not yet received");
      }
      snapshot_interfaces::msg::ImageSnapshot left_ir_snapshot;
      left_ir_snapshot.topic_name = LeftIrImageTopic();
      left_ir_snapshot.camera_info = *left_ir_camera_info_;
      left_ir_snapshot.image = *left_ir_image_;
      response.images.push_back(std::move(left_ir_snapshot));
    }

    if (IsRightIrEnabled()) {
      if (!right_ir_camera_info_) {
        return absl::UnavailableError("Right IR CameraInfo not yet received");
      }
      if (!right_ir_image_) {
        return absl::UnavailableError("Right IR Image not yet received");
      }
      snapshot_interfaces::msg::ImageSnapshot right_ir_snapshot;
      right_ir_snapshot.topic_name = RightIrImageTopic();
      right_ir_snapshot.camera_info = *right_ir_camera_info_;
      right_ir_snapshot.image = *right_ir_image_;
      response.images.push_back(std::move(right_ir_snapshot));
    }

    if (IsDepthEnabled()) {
      if (!color_camera_info_) {
        return absl::UnavailableError("Color CameraInfo not yet received.");
      }
      if (!depth_image_) {
        return absl::UnavailableError("Depth image not yet received");
      }
      snapshot_interfaces::msg::ImageSnapshot depth_snapshot;
      depth_snapshot.topic_name = DepthImageTopic();
      // The depth image is registered to the color image, so we copy it here.
      depth_snapshot.camera_info = *color_camera_info_;
      sensor_msgs::msg::Image depth_copy = *depth_image_;

      // The Orbbec camera returns the depth image as 16-bit images in
      // millimeters. We want to convert that to 32-bit float (meters) for
      // Flowstate.
      depth_snapshot.image.header = depth_copy.header;
      depth_snapshot.image.height = depth_copy.height;
      depth_snapshot.image.width = depth_copy.width;
      depth_snapshot.image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      depth_snapshot.image.is_bigendian = false;
      depth_snapshot.image.step = 4 * depth_snapshot.image.width;
      depth_snapshot.image.data.resize(depth_snapshot.image.step *
                                       depth_snapshot.image.height);
      // Use OpenCV's amazingly optimized implementation to do the conversion
      const cv::Mat depth_unsigned(depth_copy.height, depth_copy.width, CV_16U,
                                   depth_copy.data.data());
      cv::Mat depth_float(depth_snapshot.image.height,
                          depth_snapshot.image.width, CV_32F,
                          depth_snapshot.image.data.data());
      depth_unsigned.convertTo(depth_float, CV_32F, 0.001);

      response.images.push_back(std::move(depth_snapshot));
    }
  }
  RCLCPP_INFO(get_logger(), "snapshot IR-color dt = %.6f",
              rclcpp::Time(color_image_->header.stamp).seconds() -
                  rclcpp::Time(left_ir_image_->header.stamp).seconds());
  return response;
}

// If the extrinsics transforms have not yet been populated, use TF
// to query them. They should be published shortly after the Orbbec
// node starts running.
absl::Status AdapterNode::PopulateExtrinsicsIfNeeded() {
  if (color_transform_ && right_ir_transform_ && left_ir_transform_) {
    return absl::OkStatus();
  }

  if (!tf_buffer_) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  if (software_trigger_client_) {
    auto trigger_req = std::make_shared<std_srvs::srv::SetBool::Request>();
    trigger_req->data = true;
    software_trigger_client_->async_send_request(trigger_req);
  }

  const std::string color_frame =
      absl::StrFormat("orbbec_%s_color_optical_frame", serial_);
  const std::string right_ir_frame =
      absl::StrFormat("orbbec_%s_right_ir_optical_frame", serial_);
  const std::string left_ir_frame =
      absl::StrFormat("orbbec_%s_left_ir_optical_frame", serial_);

  try {
    color_transform_ = std::make_unique<geometry_msgs::msg::TransformStamped>(
        tf_buffer_->lookupTransform(color_frame, color_frame,
                                    tf2::TimePointZero,
                                    tf2::durationFromSec(0.5)));
    right_ir_transform_ =
        std::make_unique<geometry_msgs::msg::TransformStamped>(
            tf_buffer_->lookupTransform(color_frame, right_ir_frame,
                                        tf2::TimePointZero,
                                        tf2::durationFromSec(0.5)));
    left_ir_transform_ = std::make_unique<geometry_msgs::msg::TransformStamped>(
        tf_buffer_->lookupTransform(color_frame, left_ir_frame,
                                    tf2::TimePointZero,
                                    tf2::durationFromSec(0.5)));
  } catch (const tf2::TransformException& ex) {
    return absl::UnavailableError(
        absl::StrFormat("TF exception: %s", ex.what()));
  }

  RCLCPP_INFO(get_logger(),
              "color extrinsics: [%.4f, %.4f, %.4f], [%.4f, %.4f, %.4f, %.4f]",
              color_transform_->transform.translation.x,
              color_transform_->transform.translation.y,
              color_transform_->transform.translation.z,
              color_transform_->transform.rotation.x,
              color_transform_->transform.rotation.y,
              color_transform_->transform.rotation.z,
              color_transform_->transform.rotation.w);

  RCLCPP_INFO(
      get_logger(),
      "right_ir extrinsics: [%.4f, %.4f, %.4f], [%.4f, %.4f, %.4f, %.4f]",
      right_ir_transform_->transform.translation.x,
      right_ir_transform_->transform.translation.y,
      right_ir_transform_->transform.translation.z,
      right_ir_transform_->transform.rotation.x,
      right_ir_transform_->transform.rotation.y,
      right_ir_transform_->transform.rotation.z,
      right_ir_transform_->transform.rotation.w);

  RCLCPP_INFO(
      get_logger(),
      "left_ir extrinsics: [%.4f, %.4f, %.4f], [%.4f, %.4f, %.4f, %.4f]",
      left_ir_transform_->transform.translation.x,
      left_ir_transform_->transform.translation.y,
      left_ir_transform_->transform.translation.z,
      left_ir_transform_->transform.rotation.x,
      left_ir_transform_->transform.rotation.y,
      left_ir_transform_->transform.rotation.z,
      left_ir_transform_->transform.rotation.w);

  tf_listener_.reset();
  tf_buffer_.reset();

  return absl::OkStatus();
}

std::string AdapterNode::OrbbecNodeNamespace() {
  return std::string(kOrbbecNodeNamespacePrefix) + serial_;
}

rclcpp::NodeOptions AdapterNode::CreateOrbbecNodeOptions(
    const std::string& serial) {
  const std::string camera_name = std::string("orbbec_") + serial;

  rclcpp::NodeOptions options =
      rclcpp::NodeOptions()
          .use_intra_process_comms(true)
          .append_parameter_override(
              rclcpp::Parameter("camera_name", camera_name))
          .append_parameter_override(rclcpp::Parameter("serial_number", serial))
          .append_parameter_override(
              rclcpp::Parameter("sync_mode", "SOFTWARE_TRIGGERING"))
          .append_parameter_override(
              rclcpp::Parameter("software_trigger_enabled", false))
          .append_parameter_override(
              rclcpp::Parameter("software_trigger_period", 200))
          .append_parameter_override(
              rclcpp::Parameter("enable_frame_sync", true))
          .append_parameter_override(
              rclcpp::Parameter("frame_aggregate_mode", "full_frame"))
          .append_parameter_override(
              rclcpp::Parameter("diagnostic_period", -1.0))
          .append_parameter_override(
              rclcpp::Parameter("enumerate_net_device", true));

  if (IsRgbEnabled()) {
    options =
        options
            .append_parameter_override(
                rclcpp::Parameter("color_fps", static_cast<int>(fps_)))
            .append_parameter_override(rclcpp::Parameter("color_format", "RGB"))
            .append_parameter_override(rclcpp::Parameter("color_width", 1280))
            .append_parameter_override(rclcpp::Parameter("color_height", 800))
            .append_parameter_override(rclcpp::Parameter("color_sharpness", 50))
            .append_parameter_override(rclcpp::Parameter("enable_color", true));
  } else {
    options = options.append_parameter_override(
        rclcpp::Parameter("enable_color", false));
  }

  if (IsDepthEnabled()) {
    options =
        options
            .append_parameter_override(
                rclcpp::Parameter("depth_fps", static_cast<int>(fps_)))
            .append_parameter_override(rclcpp::Parameter("enable_depth", true));
  } else {
    options = options.append_parameter_override(
        rclcpp::Parameter("enable_depth", false));
  }

  if (IsLeftIrEnabled()) {
    options =
        options
            .append_parameter_override(
                rclcpp::Parameter("left_ir_fps", static_cast<int>(fps_)))
            .append_parameter_override(
                rclcpp::Parameter("left_ir_format", "Y8"))
            .append_parameter_override(rclcpp::Parameter("left_ir_width", 1280))
            .append_parameter_override(rclcpp::Parameter("left_ir_height", 800))
            .append_parameter_override(
                rclcpp::Parameter("enable_left_ir", true));
  } else {
    options = options.append_parameter_override(
        rclcpp::Parameter("enable_left_ir", false));
  }

  if (IsRightIrEnabled()) {
    options = options
                  .append_parameter_override(
                      rclcpp::Parameter("right_ir_fps", static_cast<int>(fps_)))
                  .append_parameter_override(
                      rclcpp::Parameter("right_ir_format", "Y8"))
                  .append_parameter_override(
                      rclcpp::Parameter("right_ir_width", 1280))
                  .append_parameter_override(
                      rclcpp::Parameter("right_ir_height", 800))
                  .append_parameter_override(
                      rclcpp::Parameter("enable_right_ir", true));
  } else {
    options = options.append_parameter_override(
        rclcpp::Parameter("enable_right_ir", false));
  }

  bool enable_laser = true;
  get_parameter<bool>("enable_laser", enable_laser);
  options = options.append_parameter_override(
      rclcpp::Parameter("enable_laser", enable_laser));

  if (IsDepthEnabled() && IsRgbEnabled()) {
    RCLCPP_INFO(get_logger(), "Enabling depth registration");
    // The Gemini 335 hardware cannot handle 1280x800 registration in hardware
    // so we need to explicitly ask it to do software depth registration.
    options =
        options
            .append_parameter_override(
                rclcpp::Parameter("depth_registration", true))
            .append_parameter_override(rclcpp::Parameter("align_mode", "SW"));
  } else {
    RCLCPP_INFO(
        get_logger(),
        "Depth registration not enabled: requires depth and color at startup");
  }
  return options;
}

bool AdapterNode::IsRgbEnabled() const {
  bool enable_rgb = true;
  get_parameter<bool>("enable_rgb", enable_rgb);
  return enable_rgb;
}

bool AdapterNode::IsDepthEnabled() const {
  bool enable_depth = true;
  get_parameter<bool>("enable_depth", enable_depth);
  return enable_depth;
}

bool AdapterNode::IsLeftIrEnabled() const {
  bool enable_left_ir = true;
  get_parameter<bool>("enable_left_ir", enable_left_ir);
  return enable_left_ir;
}

bool AdapterNode::IsRightIrEnabled() const {
  bool enable_right_ir = true;
  get_parameter<bool>("enable_right_ir", enable_right_ir);
  return enable_right_ir;
}

void AdapterNode::CreateOrbbecNode() {
  absl::MutexLock lock(&SpawnerNode::s_discovery_mutex);
  RCLCPP_INFO(get_logger(), "Creating OBCameraNodeDriver for %s",
              serial_.c_str());
  if (orbbec_node_) {
    RCLCPP_INFO(get_logger(), "Orbbec node already existed. Destroying it");
    DestroyOrbbecNode();
  }

  try {
    orbbec_node_ = std::make_unique<orbbec_camera::OBCameraNodeDriver>(
        std::string(kOrbbecNodeName), OrbbecNodeNamespace(),
        CreateOrbbecNodeOptions(serial_));
  } catch (const std::exception& e) {
    RCLCPP_FATAL(get_logger(), "Failed to create OBCameraNodeDriver: %s", e.what());
  }
  orbbec_thread_ = std::make_unique<std::thread>([this]() {
    this->orbbec_executor_ =
        std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    this->orbbec_executor_->add_node(
        this->orbbec_node_->get_node_base_interface());
    this->orbbec_executor_->spin();
  });
  RCLCPP_INFO(get_logger(), "Sleeping a bit to allow Orbbec thread to start");
  rclcpp::sleep_for(std::chrono::seconds(5));
  this->warmup_snapshot_count_ = 0;
  initial_snapshot_timer_ =
      create_wall_timer(std::chrono::milliseconds(500), [this]() {
        absl::StatusOr<snapshot_interfaces::srv::Snapshot::Response> response =
            this->BuildSnapshotResponse();
        if (!response.ok()) {
          RCLCPP_ERROR(get_logger(), "Initial snapshot %d error: %s",
                       this->warmup_snapshot_count_,
                       std::string(response.status().message()).c_str());
        } else {
          RCLCPP_INFO(get_logger(), "Initial snapshot %d OK", this->warmup_snapshot_count_);
        }
        this->warmup_snapshot_count_++;
        if (this->warmup_snapshot_count_ > 20) {
          this->initial_snapshot_timer_->cancel();
        }
      });
}

void AdapterNode::DestroyOrbbecNode() {
  RCLCPP_INFO(get_logger(), "Destroying orbbec_node");
  orbbec_node_.reset();
  RCLCPP_INFO(get_logger(), "Waiting a few seconds");
  rclcpp::sleep_for(std::chrono::seconds(10));
  if (orbbec_executor_) {
    orbbec_executor_->cancel();
    orbbec_executor_.reset();
  }
  if (!orbbec_thread_) {
    RCLCPP_ERROR(get_logger(), "Expected orbbec_thread to exist!");
  } else {
    RCLCPP_INFO(get_logger(), "Joining orbbec_thread");
    orbbec_thread_->join();
    RCLCPP_INFO(get_logger(), "Done joining orbbec_thread");
    orbbec_thread_.reset();
  }
}

}  // namespace flowstate_orbbec
