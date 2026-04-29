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
  set_parameters_callback_handle_ =
      this->add_on_set_parameters_callback(std::bind(
          &AdapterNode::SetParametersCallback, this, std::placeholders::_1));

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
  declare_parameter<double>("projector_brightness", capture_params_.projector_brightness);
}

rcl_interfaces::msg::SetParametersResult AdapterNode::SetParametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  
  absl::MutexLock lock(&capture_params_mutex_);
  
  auto goal_msg = ensenso_camera_msgs::action::SetParameter::Goal();
  bool requires_camera_update = false;

  for (const auto& param : parameters) {
    RCLCPP_INFO_STREAM(get_logger(), "AdapterNode: Setting parameter '"
                                         << param.get_name() << "' to '"
                                         << param.value_to_string() << "'");

    ensenso_camera_msgs::msg::Parameter ensenso_param;
    ensenso_camera_msgs::msg::Parameter auto_toggle_param;

    if (param.get_name() == "exposure_time") {
      capture_params_.exposure_time = param.as_double();
      
      auto_toggle_param.key = ensenso_camera_msgs::msg::Parameter::AUTO_EXPOSURE;
      auto_toggle_param.bool_value = false;
      goal_msg.parameters.push_back(auto_toggle_param);

      ensenso_param.key = ensenso_camera_msgs::msg::Parameter::EXPOSURE;
      ensenso_param.float_value = capture_params_.exposure_time;
      goal_msg.parameters.push_back(ensenso_param);
      
      requires_camera_update = true;

    } else if (param.get_name() == "gain") {
      capture_params_.gain = param.as_double();
      
      auto_toggle_param.key = ensenso_camera_msgs::msg::Parameter::AUTO_GAIN;
      auto_toggle_param.bool_value = false;
      goal_msg.parameters.push_back(auto_toggle_param);

      ensenso_param.key = ensenso_camera_msgs::msg::Parameter::GAIN;
      ensenso_param.float_value = capture_params_.gain;
      goal_msg.parameters.push_back(ensenso_param);
      
      requires_camera_update = true;

    } else if (param.get_name() == "gamma") {
      capture_params_.gamma = param.as_double();
      
      ensenso_param.key = "Gamma"; 
      ensenso_param.float_value = capture_params_.gamma;
      goal_msg.parameters.push_back(ensenso_param);
      
      requires_camera_update = true;

    } else if (param.get_name() == "projector_brightness") {
      capture_params_.projector_brightness = param.as_double();
      
      ensenso_param.key = ensenso_camera_msgs::msg::Parameter::PROJECTOR;
      ensenso_param.bool_value = (capture_params_.projector_brightness > 0.0);
      goal_msg.parameters.push_back(ensenso_param);
      
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
  RCLCPP_INFO(get_logger(), "Triggering Ensenso %s...", only_info_needed ? "for CameraInfo" : "Snapshot");

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

  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    return absl::InternalError("Action failed.");
  }

  CaptureData result_data;
  
  if (!only_info_needed) {
    if (!result->left_rectified_images.empty()) {
      result_data.left_image = std::make_unique<sensor_msgs::msg::Image>(std::move(result->left_rectified_images[0]));
    }
    result_data.depth_image = std::make_unique<sensor_msgs::msg::Image>(std::move(result->depth_image));
  }
  
  result_data.left_camera_info = std::make_unique<sensor_msgs::msg::CameraInfo>(std::move(result->left_rectified_camera_info));
  result_data.depth_camera_info = std::make_unique<sensor_msgs::msg::CameraInfo>(std::move(result->depth_image_info));

  {
    absl::MutexLock lock(&data_mutex_);
    data_.left_camera_info = std::make_unique<sensor_msgs::msg::CameraInfo>(*result_data.left_camera_info);
    data_.depth_camera_info = std::make_unique<sensor_msgs::msg::CameraInfo>(*result_data.depth_camera_info);
    if (!only_info_needed) {
      if (result_data.left_image) {
        data_.left_image = std::make_unique<sensor_msgs::msg::Image>(*result_data.left_image);
      }
      if (result_data.depth_image) {
        data_.depth_image = std::make_unique<sensor_msgs::msg::Image>(*result_data.depth_image);
      }
    }
  }

  return result_data;
}

absl::StatusOr<snapshot_interfaces::srv::Describe::Response>
AdapterNode::BuildDescribeResponse() {
  snapshot_interfaces::srv::Describe::Response response;
  std::unique_ptr<sensor_msgs::msg::CameraInfo> info_copy;
  std::unique_ptr<sensor_msgs::msg::CameraInfo> depth_info_copy;

  {
    absl::MutexLock lock(&data_mutex_);
    if (data_.left_camera_info) {
      info_copy = std::make_unique<sensor_msgs::msg::CameraInfo>(*data_.left_camera_info);
    }
    if (data_.depth_camera_info) {
      depth_info_copy = std::make_unique<sensor_msgs::msg::CameraInfo>(*data_.depth_camera_info);
    }
  }

  if (info_copy == nullptr || depth_info_copy == nullptr) {
    RCLCPP_INFO(get_logger(), "No cached CameraInfo available, triggering warm-up capture...");
    auto capture_data = Capture(true);
    if (!capture_data.ok()) {
      return capture_data.status();
    }
    info_copy = std::move(capture_data->left_camera_info);
    depth_info_copy = std::move(capture_data->depth_camera_info);
  }

  response.sensors.push_back(
      BuildSensorInformation(*info_copy, "left_rectified", ColorImageTopic()));
  
  response.sensors.push_back(
      BuildSensorInformation(*depth_info_copy, "depth", DepthImageTopic()));

  return response;
}

absl::StatusOr<snapshot_interfaces::srv::Snapshot::Response>
AdapterNode::BuildSnapshotResponse() {
  snapshot_interfaces::srv::Snapshot::Response response;

  auto capture_data = Capture(false);
  if (!capture_data.ok()) {
    return capture_data.status();
  }

  snapshot_interfaces::msg::ImageSnapshot left_snapshot;
  left_snapshot.topic_name = ColorImageTopic();
  left_snapshot.camera_info = std::move(*capture_data->left_camera_info);
  left_snapshot.image = std::move(*capture_data->left_image);
  response.images.push_back(std::move(left_snapshot));

  snapshot_interfaces::msg::ImageSnapshot depth_snapshot;
  depth_snapshot.topic_name = DepthImageTopic();
  depth_snapshot.camera_info = std::move(*capture_data->depth_camera_info);
  depth_snapshot.image = std::move(*capture_data->depth_image);
  response.images.push_back(std::move(depth_snapshot));

  return response;
}

}  // namespace flowstate_ensenso
