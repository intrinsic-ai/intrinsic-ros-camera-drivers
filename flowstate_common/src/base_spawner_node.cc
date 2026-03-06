#include "flowstate_common/base_spawner_node.h"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "rclcpp/rclcpp.hpp"
#include "snapshot_interfaces/msg/discovered_camera.hpp"

namespace flowstate_common {

using snapshot_interfaces::srv::Discover;

BaseSpawnerNode::BaseSpawnerNode(const std::string& node_name,
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
        RCLCPP_INFO_ONCE(get_logger(),
                         "Discover service called (logging once)");

        absl::MutexLock lock(&this->nodes_mutex_);
        for (const auto& node : spawned_nodes_) {
          snapshot_interfaces::msg::DiscoveredCamera camera;
          camera.driver_type = this->driver_type_;
          camera.camera_id = node->GetSerial();
          response->cameras.push_back(camera);
        }
        response->success = true;
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

void BaseSpawnerNode::CleanupDeadNodes(
    const std::vector<std::string>& current_serials) {
  auto it = spawned_nodes_.begin();
  while (it != spawned_nodes_.end()) {
    const std::string& node_serial = (*it)->GetSerial();

    bool has_exited = (*it)->HasExitedThread();
    bool is_unplugged =
        std::find(current_serials.begin(), current_serials.end(),
                  node_serial) == current_serials.end();

    if (has_exited || is_unplugged) {
      if (has_exited) {
        RCLCPP_INFO(get_logger(), "Camera %s thread exited. Removing node.",
                    node_serial.c_str());
      } else {
        RCLCPP_WARN(get_logger(),
                    "Camera %s physically disconnected. Removing node.",
                    node_serial.c_str());
      }

      it = spawned_nodes_.erase(it);
    } else {
      ++it;
    }
  }
}

void BaseSpawnerNode::UpdateCameras() {
  std::vector<std::string> current_serials = GetSerials();

  absl::MutexLock lock(&nodes_mutex_);

  CleanupDeadNodes(current_serials);

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
