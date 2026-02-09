#include "flowstate_common/camera_adapter_node.h"

#include "rclcpp/rclcpp.hpp"

namespace flowstate_common {

CameraAdapterNode::CameraAdapterNode(const std::string& serial,
                                     const std::string& ip_address,
                                     const std::string& node_name_prefix)
    : Node(node_name_prefix + "_" + serial),
      serial_(serial),
      ip_address_(ip_address) {}

void CameraAdapterNode::CreateFlowstateServices() {
  describe_service_ = create_service<snapshot_interfaces::srv::Describe>(
      "~/describe",
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
             const std::shared_ptr<snapshot_interfaces::srv::Describe::Request>
                 request,
             const std::shared_ptr<snapshot_interfaces::srv::Describe::Response>
                 response) {
        this->DescribeCallback(request_header, request, response);
      });

  snapshot_service_ = create_service<snapshot_interfaces::srv::Snapshot>(
      "~/snapshot",
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
             const std::shared_ptr<snapshot_interfaces::srv::Snapshot::Request>
                 request,
             const std::shared_ptr<snapshot_interfaces::srv::Snapshot::Response>
                 response) {
        this->SnapshotCallback(request_header, request, response);
      });
}

void CameraAdapterNode::StartExecutorThread() {
  thread_ = std::thread([this]() {
    const absl::Status status = this->Main();
    if (!status.ok()) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "node thread error: " << status);
    }
    exited_thread_ = true;
  });
}

void CameraAdapterNode::DescribeCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<snapshot_interfaces::srv::Describe::Request>,
    const std::shared_ptr<snapshot_interfaces::srv::Describe::Response>
        response) {  
  if (!BuildDescribeResponse(*response)) {
    if (response->error_message.empty()) {
        response->error_message = "Failed to build describe response (data not ready)";
    }
    response->success = false;
    RCLCPP_ERROR(get_logger(), response->error_message.c_str());
    return;
  }

  response->success = true;
}

void CameraAdapterNode::AppendSensorDescription(
    snapshot_interfaces::srv::Describe::Response& response,
    const sensor_msgs::msg::CameraInfo& info,
    const std::string& sensor_name,
    const std::string& topic_name) {
  
  snapshot_interfaces::msg::SensorInfo sensor_info;
  sensor_info.sensor_name = sensor_name;
  sensor_info.topic_name = topic_name;
  sensor_info.sensor_type = snapshot_interfaces::msg::SensorInfo::IMAGE;
  
  sensor_info.camera_t_sensor.transform.rotation.w = 1.0; // todo: get static transform
  
  sensor_info.info.push_back(info);
  response.sensors.push_back(std::move(sensor_info));
}

void CameraAdapterNode::SnapshotCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<snapshot_interfaces::srv::Snapshot::Request>,
    const std::shared_ptr<snapshot_interfaces::srv::Snapshot::Response>
        response) {
  if (!BuildSnapshotResponse(*response)) {
    if (response->error_message.empty()) {
        response->error_message = "Failed to capture snapshot (data not ready)";
    }
    response->success = false;
    RCLCPP_ERROR(get_logger(), response->error_message.c_str());
    return;
  }

  response->success = true;
}

}  // namespace flowstate_common
