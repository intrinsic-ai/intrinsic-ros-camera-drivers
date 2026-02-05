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
  absl::MutexLock lock(&camera_info_mutex_);
  if (!color_camera_info_) {
    response->error_message = "CameraInfo not yet received from camera";
    response->success = false;
    RCLCPP_ERROR(get_logger(), response->error_message.c_str());
    return;
  }

  if (!BuildDescribeResponse(*response)) {
    response->error_message = "Failed to build describe response";
    response->success = false;
    RCLCPP_ERROR(get_logger(), response->error_message.c_str());
    return;
  }

  response->success = true;
}

void CameraAdapterNode::SnapshotCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<snapshot_interfaces::srv::Snapshot::Request>
        /*request*/,
    const std::shared_ptr<snapshot_interfaces::srv::Snapshot::Response>
        response) {
  {
    absl::MutexLock lock(&camera_info_mutex_);
    if (!color_camera_info_) {
      response->error_message = "CameraInfo not yet received";
      response->success = false;
      RCLCPP_ERROR(get_logger(), response->error_message.c_str());
      return;
    }
  }

  absl::MutexLock lock(&image_mutex_);
  if (!color_image_) {
    response->error_message = "images not yet received from camera";
    response->success = false;
    RCLCPP_ERROR(get_logger(), response->error_message.c_str());
    return;
  }

  if (!BuildSnapshotResponse(*response)) {
    response->error_message = "Failed to build snapshot response";
    response->success = false;
    RCLCPP_ERROR(get_logger(), response->error_message.c_str());
    return;
  }

  response->success = true;
}

}  // namespace flowstate_common
