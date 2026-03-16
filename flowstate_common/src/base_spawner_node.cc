#include "flowstate_common/base_spawner_node.h"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "rclcpp/rclcpp.hpp"
#include "snapshot_interfaces/msg/discovered_camera.hpp"
#include "snapshot_interfaces/msg/discovery_request.hpp"
#include "snapshot_interfaces/msg/discovery_response.hpp"

namespace flowstate_common {

struct DiscoveryResponse {
  bool success = true;
  std::string error_message;
  std::vector<snapshot_interfaces::msg::DiscoveredCamera> cameras;
};

BaseSpawnerNode::BaseSpawnerNode(const std::string& node_name,
                                 const std::string& driver_type,
                                 std::chrono::duration<double> update_period,
                                 const rclcpp::NodeOptions& options)
    : Node(node_name, options), driver_type_(driver_type) {
  discovery_pub_ =
      create_publisher<snapshot_interfaces::msg::DiscoveryResponse>(
          "/cameras/discovery_responses", 10);

  discovery_sub_ = create_subscription<
      snapshot_interfaces::msg::DiscoveryRequest>(
      "/cameras/discovery_request", 10,
      [this](const snapshot_interfaces::msg::DiscoveryRequest::
                 ConstSharedPtr /*msg*/) {
        RCLCPP_INFO(
            get_logger(),
            "Received DiscoveryRequest on topic '/cameras/discovery_request'.");
        snapshot_interfaces::msg::DiscoveryResponse response;

        {
          absl::MutexLock lock(&this->nodes_mutex_);
          for (const auto& node : spawned_nodes_) {
            if (node) {
              snapshot_interfaces::msg::DiscoveredCamera camera;
              camera.driver_type = this->driver_type_;
              camera.camera_id = node->GetSerial();
              response.cameras.push_back(camera);
            }
          }
        }

        discovery_pub_->publish(response);
      });

  // Initialize Timer
  timer_ =
      create_wall_timer(update_period, [this]() { this->UpdateCameras(); });
}

BaseSpawnerNode::~BaseSpawnerNode() {
  if (timer_) {
    timer_.reset();
  }
  RCLCPP_INFO(get_logger(), "Camera SpawnerNode shutdown complete");
}

bool BaseSpawnerNode::IsAlreadySpawned(const std::string& serial) const {
  for (const auto& node : spawned_nodes_) {
    // Uses the helper from BaseAdapterNode
    if (node && serial == node->GetSerial()) {
      return true;
    }
  }
  return false;
}

void BaseSpawnerNode::CleanupExitedNodes() {
  auto it = spawned_nodes_.begin();
  while (it != spawned_nodes_.end()) {
    // Uses the helper from BaseAdapterNode
    if ((*it)->HasExitedThread()) {
      RCLCPP_INFO(get_logger(), "Camera %s has exited. Removing it.",
                  (*it)->GetSerial().c_str());
      it = spawned_nodes_.erase(it);
    } else {
      ++it;
    }
  }
}

void BaseSpawnerNode::UpdateCameras() {
  std::vector<std::string> current_serials = GetSerials();

  absl::MutexLock lock(&nodes_mutex_);

  CleanupExitedNodes();

  std::vector<std::string> serials_to_spawn;
  for (const auto& serial : current_serials) {
    if (IsAlreadySpawned(serial)) continue;
    serials_to_spawn.push_back(serial);
  }

  if (!serials_to_spawn.empty()) {
    auto new_nodes = SpawnNodes(serials_to_spawn);
    for (auto& new_node : new_nodes) {
      if (new_node) {
        spawned_nodes_.push_back(std::move(new_node));
      }
    }
  }
}

}  // namespace flowstate_common
