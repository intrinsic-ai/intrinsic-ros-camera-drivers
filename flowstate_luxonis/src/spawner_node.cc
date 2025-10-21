#include "flowstate_luxonis/spawner_node.h"

#include <memory>

#include "depthai/device/Device.hpp"
#include "depthai/depthai.hpp"
// #include "flowstate_orbbec/adapter_node.h"
// #include "orbbec_camera/ob_camera_node_driver.h"
#include "rclcpp/rclcpp.hpp"
#include "snapshot_interfaces/msg/discovered_camera.hpp"
#include "snapshot_interfaces/srv/discover.hpp"

namespace flowstate_luxonis {

using snapshot_interfaces::srv::Discover;

SpawnerNode::SpawnerNode()
    : Node(std::string("luxonis_spawner")) {
  discover_service_ = create_service<Discover>(
      std::string("/cameras/discover"),
      [this](const std::shared_ptr<rmw_request_id_t>,
             const std::shared_ptr<Discover::Request>,
             const std::shared_ptr<Discover::Response> response) {
        RCLCPP_INFO(get_logger(), "Discover service called");
        absl::MutexLock lock(&this->serials_mutex_);
        for (const std::string& serial : serials_) {
          snapshot_interfaces::msg::DiscoveredCamera camera;
          camera.driver_type = "luxonis";
          camera.camera_id = serial;
          response->cameras.push_back(camera);
        }
        response->success = true;
      });
  timer_ = create_wall_timer(std::chrono::seconds(10),
                             [this]() { this->UpdateCameras(); });
  UpdateCameras();
}

std::string SpawnerNode::DeviceStateToString(XLinkDeviceState_t state) {
  switch (state) {
    case X_LINK_ANY_STATE:
      return std::string("any_state");
    case X_LINK_BOOTED:
      return std::string("booted");
    case X_LINK_UNBOOTED:
      return std::string("unbooted");
    case X_LINK_BOOTLOADER:
      return std::string("bootloader");
    case X_LINK_FLASH_BOOTED:
      return std::string("flash_booted");
    default:
      return std::string("unknown");
  }
}

void SpawnerNode::UpdateCameras() {
  std::vector<dai::DeviceInfo> devices = dai::Device::getAllAvailableDevices();
  if (!devices.empty()) {
    RCLCPP_INFO(get_logger(), "Found %lu devices", devices.size());
  }
  absl::MutexLock lock(&this->serials_mutex_);
  serials_.clear();
  for (const auto& device_info : devices) {
    const std::string ip_str(device_info.name);
    const std::string serial(device_info.mxid);
    const std::string state_str(DeviceStateToString(device_info.state));

    RCLCPP_INFO(get_logger(), "  ip: %s state: %s mxid: %s", ip_str.c_str(),
                state_str.c_str(), serial.c_str());
    serials_.push_back(serial);
    if (IsAlreadySpawned(serial)) continue;
    RCLCPP_INFO(get_logger(), "Spawning it...");
    spawned_nodes_.push_back(std::make_unique<AdapterNode>(serial, ip_str));
  }

  // See if any camera nodes have crashed. If so, close them so we can respawn
  for (auto node_it = spawned_nodes_.begin();
       node_it != spawned_nodes_.end();) {
    if ((*node_it)->HasExitedThread()) {
      RCLCPP_INFO(get_logger(), "Camera %s has exited. Removing it.",
                  (*node_it)->GetSerial().c_str());
      node_it = spawned_nodes_.erase(node_it);
    } else {
      serials_.push_back((*node_it)->GetSerial());
      ++node_it;
    }
  }
}

bool SpawnerNode::IsAlreadySpawned(const std::string& serial) const {
  for (const auto& spawned_node : spawned_nodes_) {
    if (spawned_node->HasSerial(serial)) {
      return true;
    }
  }
  return false;
}

}  // namespace flowstate_luxonis
