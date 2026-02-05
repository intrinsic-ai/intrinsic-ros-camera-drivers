#include "flowstate_common/camera_spawner_node.h"

#include "rclcpp/rclcpp.hpp"

namespace flowstate_common {

CameraSpawnerNode::CameraSpawnerNode(const std::string& node_name,
                                     double discovery_interval_seconds)
    : Node(node_name) {
  discover_service_ = create_service<snapshot_interfaces::srv::Discover>(
      "discover",
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
             const std::shared_ptr<snapshot_interfaces::srv::Discover::Request>
                 request,
             const std::shared_ptr<snapshot_interfaces::srv::Discover::Response>
                 response) {
        this->DiscoverServiceCallback(request_header, request, response);
      });

  discovery_timer_ = create_wall_timer(
      std::chrono::duration<double>(discovery_interval_seconds),
      [this]() { this->DiscoveryTimer(); });

  RCLCPP_INFO(get_logger(), "Spawner node started with %.1f second discovery "
                            "interval",
              discovery_interval_seconds);
}

CameraSpawnerNode::~CameraSpawnerNode() {
  if (discovery_timer_) {
    discovery_timer_->cancel();
  }
}

void CameraSpawnerNode::RegisterDiscoveredCamera(const std::string& serial,
                                                 const std::string& model,
                                                 const std::string& ip_address) {
  auto camera_msg = snapshot_interfaces::msg::DiscoveredCamera();
  camera_msg.serial = serial;
  camera_msg.model = model;
  camera_msg.ip_address = ip_address;

  {
    absl::MutexLock lock(&cameras_mutex_);
    discovered_camera_msgs_.clear();
    discovered_camera_msgs_.push_back(camera_msg);
  }
}

void CameraSpawnerNode::DiscoveryTimer() {
  RCLCPP_INFO(get_logger(), "Running camera discovery...");
  UpdateCameraList();
}

void CameraSpawnerNode::DiscoverServiceCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<snapshot_interfaces::srv::Discover::Request>
        /*request*/,
    const std::shared_ptr<snapshot_interfaces::srv::Discover::Response>
        response) {
  absl::MutexLock lock(&cameras_mutex_);
  response->cameras = discovered_camera_msgs_;
  response->success = true;
}

}  // namespace flowstate_common
