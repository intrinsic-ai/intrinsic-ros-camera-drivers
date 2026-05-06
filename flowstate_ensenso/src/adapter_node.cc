#include "flowstate_ensenso/adapter_node.h"

#include "absl/strings/str_format.h"
#include "rclcpp/rclcpp.hpp"
#include "ensenso_camera/stereo_camera_node.h"
#include "ensenso_camera_msgs/msg/parameter.hpp"

namespace flowstate_ensenso {

AdapterNode::AdapterNode(const std::string& serial,
                         const std::vector<std::string>& locators)
    : flowstate_common::BaseAdapterNode(serial, locators, "ensenso") {
  
  std::string camera_ns = absl::StrFormat("/ensenso/camera_%s", serial_);

  rclcpp::NodeOptions ensenso_options;
  ensenso_options.append_parameter_override("serial", serial);
  ensenso_options.append_parameter_override("capture_timeout", 5000);

  ensenso_options.arguments({"--ros-args", "-r", "__ns:=" + camera_ns});
  
  ensenso_node_ = std::make_unique<ensenso_camera::StereoCameraNode>(ensenso_options);

  InitializeParameters();
  set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind_front(&AdapterNode::SetParametersCallback, this));

  CreateFlowstateServices();
  
  std::string action_name = camera_ns + "/request_data";
  request_data_client_ = rclcpp_action::create_client<ensenso_camera_msgs::action::RequestData>(
      this, action_name, callback_group_);

  std::string set_param_action_name = camera_ns + "/set_parameter";
  set_parameter_client_ = rclcpp_action::create_client<ensenso_camera_msgs::action::SetParameter>(
      this, set_param_action_name, callback_group_);

  StartExecutorThread();
}

void AdapterNode::InitializeParameters() {
  absl::MutexLock lock(&capture_params_mutex_);
  declare_parameter<double>("exposure_time", capture_params_.exposure_time);
  declare_parameter<double>("gain", capture_params_.gain);
  declare_parameter<double>("gamma", capture_params_.gamma);
  declare_parameter<bool>("projector", capture_params_.projector);
}

rcl_interfaces::msg::SetParametersResult AdapterNode::SetParametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  
  absl::MutexLock lock(&capture_params_mutex_);
  auto goal_msg = ensenso_camera_msgs::action::SetParameter::Goal();
  bool requires_camera_update = false;

  auto add_param = [&](const std::string& key, double value, const std::string& auto_key = "") {
    if (!auto_key.empty()) {
      ensenso_camera_msgs::msg::Parameter auto_toggle;
      auto_toggle.key = auto_key;
      auto_toggle.bool_value = false;
      goal_msg.parameters.push_back(auto_toggle);
    }
    ensenso_camera_msgs::msg::Parameter p;
    p.key = key;
    p.float_value = static_cast<float>(value);
    goal_msg.parameters.push_back(p);
    requires_camera_update = true;
  };

  for (const auto& param : parameters) {
    RCLCPP_INFO_STREAM(get_logger(), "AdapterNode: Setting parameter '"
                                         << param.get_name() << "' to '"
                                         << param.value_to_string() << "'");

    if (param.get_name() == "exposure_time") {
      capture_params_.exposure_time = param.as_double();
      add_param(ensenso_camera_msgs::msg::Parameter::EXPOSURE, capture_params_.exposure_time, ensenso_camera_msgs::msg::Parameter::AUTO_EXPOSURE);
    } else if (param.get_name() == "gain") {
      capture_params_.gain = param.as_double();
      add_param(ensenso_camera_msgs::msg::Parameter::GAIN, capture_params_.gain, ensenso_camera_msgs::msg::Parameter::AUTO_GAIN);
    } else if (param.get_name() == "gamma") {
      capture_params_.gamma = param.as_double();
      add_param("Gamma", capture_params_.gamma);
    } else if (param.get_name() == "projector") {
      capture_params_.projector = param.as_bool();
      ensenso_camera_msgs::msg::Parameter projector_param;
      projector_param.key = ensenso_camera_msgs::msg::Parameter::PROJECTOR;
      projector_param.bool_value = capture_params_.projector;
      goal_msg.parameters.push_back(projector_param);
      requires_camera_update = true;
    }
  }

  if (requires_camera_update) {
    if (!set_parameter_client_->action_server_is_ready()) {
      RCLCPP_WARN(get_logger(), "Ensenso set_parameter action server not ready! Settings cached but not applied.");
      result.successful = false;
      result.reason = "Action server not ready";
      return result;
    }

    auto send_goal_options = rclcpp_action::Client<ensenso_camera_msgs::action::SetParameter>::SendGoalOptions();
    set_parameter_client_->async_send_goal(goal_msg, send_goal_options);
  }

  return result;
}

std::string AdapterNode::ColorImageTopic() const {
  return absl::StrFormat("/ensenso/camera_%s/rectified/left/image", serial_);
}

std::string AdapterNode::DepthImageTopic() const {
  return absl::StrFormat("/ensenso/camera_%s/depth/image", serial_);
}

std::string AdapterNode::LeftCameraInfoTopic() const {
  return absl::StrFormat("/ensenso/camera_%s/rectified/left/camera_info", serial_);
}

std::string AdapterNode::DepthCameraInfoTopic() const {
  return absl::StrFormat("/ensenso/camera_%s/depth/camera_info", serial_);
}

absl::Status AdapterNode::Main() {
  RCLCPP_INFO(get_logger(), "AdapterNode::Main() for ensenso camera %s",
              serial_.c_str());
  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(this->get_node_base_interface());
  executor.add_node(ensenso_node_->get_node_base_interface());
  executor.spin();
  
  return absl::OkStatus();
}

absl::StatusOr<AdapterNode::CaptureData> AdapterNode::Capture(bool only_info_needed) {
  RCLCPP_INFO(get_logger(), "Triggering Ensenso for %s...", only_info_needed ? "CameraInfo" : "Snapshot");

  if (!request_data_client_->action_server_is_ready()) {
    if (!request_data_client_->wait_for_action_server(std::chrono::seconds(2))) {
      return absl::UnavailableError("Ensenso request_data action server is not ready.");
    }
  }

  auto goal_msg = ensenso_camera_msgs::action::RequestData::Goal();
  goal_msg.request_rectified_images = true;
  goal_msg.request_depth_image = true;
  goal_msg.include_results_in_response = true;
  goal_msg.publish_results = false; 
  
  auto send_goal_options = rclcpp_action::Client<ensenso_camera_msgs::action::RequestData>::SendGoalOptions();
  auto goal_handle_future = request_data_client_->async_send_goal(goal_msg, send_goal_options);

  if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    return absl::DeadlineExceededError("Action goal request timed out.");
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    return absl::InternalError("Action goal was rejected by the server.");
  }

  auto result_future = request_data_client_->async_get_result(goal_handle);
  if (result_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
    return absl::DeadlineExceededError("Action result timed out.");
  }

  auto wrapped_result = result_future.get();
  auto result = wrapped_result.result;

  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED || !result) {
    return absl::InternalError("Action failed or returned null result.");
  }

  CaptureData result_data;
  
  if (!only_info_needed) {
    if (!result->left_rectified_images.empty()) {
      result_data.left_image = std::move(result->left_rectified_images[0]);
    }
    result_data.depth_image = std::move(result->depth_image);
  }
  
  result_data.left_camera_info = std::move(result->left_rectified_camera_info);
  result_data.depth_camera_info = std::move(result->depth_image_info);

  {
    absl::MutexLock lock(&data_mutex_);
    cached_info_.left = result_data.left_camera_info;
    cached_info_.depth = result_data.depth_camera_info;
  }

  return result_data;
}

absl::StatusOr<snapshot_interfaces::srv::Describe::Response>
AdapterNode::BuildDescribeResponse() {
  snapshot_interfaces::srv::Describe::Response response;
  std::optional<sensor_msgs::msg::CameraInfo> info_copy;
  std::optional<sensor_msgs::msg::CameraInfo> depth_info_copy;

  {
    absl::MutexLock lock(&data_mutex_);
    info_copy = cached_info_.left;
    depth_info_copy = cached_info_.depth;
  }

  if (!info_copy.has_value() || !depth_info_copy.has_value()) {
    RCLCPP_INFO(get_logger(), "No cached CameraInfo available, triggering warm-up capture...");
    auto capture_data = Capture(true);
    if (!capture_data.ok()) {
      return capture_data.status();
    }
    info_copy = std::move(capture_data->left_camera_info);
    depth_info_copy = std::move(capture_data->depth_camera_info);
  }

  if (!info_copy.has_value() || !depth_info_copy.has_value()) {
    return absl::InternalError("Failed to retrieve valid CameraInfo from device.");
  }

  response.sensors.push_back(
      BuildSensorInformation(info_copy.value(), "left_rectified", ColorImageTopic()));
  
  response.sensors.push_back(
      BuildSensorInformation(depth_info_copy.value(), "depth", DepthImageTopic()));

  return response;
}

absl::StatusOr<snapshot_interfaces::srv::Snapshot::Response>
AdapterNode::BuildSnapshotResponse() {
  snapshot_interfaces::srv::Snapshot::Response response;

  auto capture_data = Capture(false);
  if (!capture_data.ok()) {
    return capture_data.status();
  }

  if (!capture_data->left_image.has_value() || !capture_data->left_camera_info.has_value()) {
    return absl::InternalError("Ensenso capture succeeded but returned missing left image/info.");
  }
  if (!capture_data->depth_image.has_value() || !capture_data->depth_camera_info.has_value()) {
    return absl::InternalError("Ensenso capture succeeded but returned missing depth image/info.");
  }

  snapshot_interfaces::msg::ImageSnapshot left_snapshot;
  left_snapshot.topic_name = ColorImageTopic();
  left_snapshot.camera_info = std::move(capture_data->left_camera_info.value());
  left_snapshot.image = std::move(capture_data->left_image.value());
  response.images.push_back(std::move(left_snapshot));

  snapshot_interfaces::msg::ImageSnapshot depth_snapshot;
  depth_snapshot.topic_name = DepthImageTopic();
  depth_snapshot.camera_info = std::move(capture_data->depth_camera_info.value());
  depth_snapshot.image = std::move(capture_data->depth_image.value());
  response.images.push_back(std::move(depth_snapshot));

  return response;
}

}  // namespace flowstate_ensenso
