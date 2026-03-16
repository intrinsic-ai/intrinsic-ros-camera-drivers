#include "flowstate_common/base_adapter_node.h"

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "rclcpp/rclcpp.hpp"
#include "snapshot_interfaces/msg/sensor_info.hpp"
#include "snapshot_interfaces/srv/describe.hpp"
#include "snapshot_interfaces/srv/snapshot.hpp"

namespace flowstate_common {

BaseAdapterNode::BaseAdapterNode(const std::string& serial,
                                 const std::vector<std::string>& locators,
                                 const std::string& node_name_prefix)
    : Node(node_name_prefix + "_" + serial),
      serial_(serial),
      locators_(locators) {}

void BaseAdapterNode::CreateFlowstateServices() {
  callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  describe_service_ = create_service<snapshot_interfaces::srv::Describe>(
      "~/describe",
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
             const std::shared_ptr<snapshot_interfaces::srv::Describe::Request>
                 request,
             const std::shared_ptr<snapshot_interfaces::srv::Describe::Response>
                 response) {
        this->DescribeCallback(request_header, request, response);
      },
      rclcpp::ServicesQoS(), callback_group_);

  snapshot_service_ = create_service<snapshot_interfaces::srv::Snapshot>(
      "~/snapshot",
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
             const std::shared_ptr<snapshot_interfaces::srv::Snapshot::Request>
                 request,
             const std::shared_ptr<snapshot_interfaces::srv::Snapshot::Response>
                 response) {
        this->SnapshotCallback(request_header, request, response);
      },
      rclcpp::ServicesQoS(), callback_group_);
}

BaseAdapterNode::~BaseAdapterNode() {
  if (thread_.joinable()) {
    thread_.join();
  }
}

void BaseAdapterNode::StartExecutorThread() {
  thread_ = std::thread([this]() {
    const absl::Status status = this->Main();
    if (!status.ok()) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "node thread error: " << status);
    }
    exited_thread_ = true;
  });
}

void BaseAdapterNode::DescribeCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<snapshot_interfaces::srv::Describe::Request>,
    const std::shared_ptr<snapshot_interfaces::srv::Describe::Response>
        response) {
  RCLCPP_INFO(get_logger(), "=== DESCRIBE SERVICE ===");

  absl::StatusOr<snapshot_interfaces::srv::Describe::Response>
      describe_response = BuildDescribeResponse();

  if (!describe_response.ok()) {
    response->success = false;
    response->error_message = std::string(describe_response.status().message());
    RCLCPP_ERROR(get_logger(), response->error_message.c_str());
    return;
  }

  *response = std::move(describe_response).value();
  response->success = true;
}

snapshot_interfaces::msg::SensorInfo BaseAdapterNode::BuildSensorInformation(
    const sensor_msgs::msg::CameraInfo& camera_info,
    const std::string& sensor_name, const std::string& topic_name) {
  snapshot_interfaces::msg::SensorInfo sensor_info;
  sensor_info.sensor_name = sensor_name;
  sensor_info.topic_name = topic_name;
  sensor_info.sensor_type = snapshot_interfaces::msg::SensorInfo::IMAGE;

  sensor_info.camera_t_sensor.transform.rotation.w =
      1.0;  // todo: get static transform

  sensor_info.info.push_back(camera_info);
  return sensor_info;
}

snapshot_interfaces::msg::SensorInfo BaseAdapterNode::BuildSensorInformation(
    const sensor_msgs::msg::CameraInfo& camera_info,
    const std::string& sensor_name, const std::string& topic_name,
    const geometry_msgs::msg::Transform& camera_t_sensor) {
  snapshot_interfaces::msg::SensorInfo sensor_info =
      BuildSensorInformation(camera_info, sensor_name, topic_name);
  sensor_info.camera_t_sensor = camera_t_sensor;
  return sensor_info;
}

void BaseAdapterNode::SnapshotCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<snapshot_interfaces::srv::Snapshot::Request>,
    const std::shared_ptr<snapshot_interfaces::srv::Snapshot::Response>
        response) {
  RCLCPP_INFO(get_logger(), "=== SNAPSHOT SERVICE ===");

  absl::StatusOr<snapshot_interfaces::srv::Snapshot::Response>
      snapshot_response = BuildSnapshotResponse();

  if (!snapshot_response.ok()) {
    response->success = false;
    response->error_message = std::string(snapshot_response.status().message());
    RCLCPP_ERROR(get_logger(), response->error_message.c_str());
    return;
  }

  *response = std::move(snapshot_response).value();
  response->success = true;
}

}  // namespace flowstate_common
