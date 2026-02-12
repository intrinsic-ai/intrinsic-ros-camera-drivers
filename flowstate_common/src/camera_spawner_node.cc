#include "flowstate_common/camera_spawner_node.h"
#include "snapshot_interfaces/msg/discovered_camera.hpp"

namespace flowstate_common {

using snapshot_interfaces::srv::Discover;

CameraSpawnerNode::CameraSpawnerNode(const std::string& node_name,
                                 const std::string& driver_type,
                                 std::chrono::duration<double> update_period,
                                 const rclcpp::NodeOptions& options)
    : Node(node_name, options), driver_type_(driver_type) {
  
  // Initialize Discover Service
  discover_service_ = create_service<Discover>(
      "/cameras/discover",
      [this](const std::shared_ptr<rmw_request_id_t>,
             const std::shared_ptr<Discover::Request>,
             const std::shared_ptr<Discover::Response> response) {
        RCLCPP_INFO_ONCE(get_logger(), "Discover service called (logging once)");
        
        absl::MutexLock lock(&this->discovery_mutex_);
        for (const std::string& serial : discovered_serials_) {
          snapshot_interfaces::msg::DiscoveredCamera camera;
          camera.driver_type = this->driver_type_;
          camera.camera_id = serial;
          response->cameras.push_back(camera);
        }
        response->success = true;
      });

  // Initialize Timer
  timer_ = create_wall_timer(update_period,
                             [this]() { this->UpdateCameras(); });
}

CameraSpawnerNode::~CameraSpawnerNode() {
  if (timer_) {
    timer_.reset();
  }
}

void CameraSpawnerNode::SetDiscoveredSerials(const std::vector<std::string>& serials) {
  absl::MutexLock lock(&discovery_mutex_);
  discovered_serials_ = serials;
}

bool CameraSpawnerNode::IsAlreadySpawned(const std::string& serial) const {
  for (const auto& node : spawned_nodes_) {
    // Uses the helper from CameraAdapterNode
    if (node && node->HasSerial(serial)) return true;
  }
  return false;
}

void CameraSpawnerNode::CleanupExitedNodes() {
  auto it = spawned_nodes_.begin();
  while (it != spawned_nodes_.end()) {
    // Uses the helper from CameraAdapterNode
    if ((*it)->HasExitedThread()) {
      RCLCPP_INFO(get_logger(), "Camera %s has exited. Removing it.",
                  (*it)->GetSerial().c_str());
      it = spawned_nodes_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace flowstate_common