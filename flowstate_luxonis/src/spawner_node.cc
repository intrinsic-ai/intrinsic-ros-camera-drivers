#include "flowstate_luxonis/spawner_node.h"

#include <memory>

#include "depthai/device/Device.hpp"
#include "depthai/depthai.hpp"
#include "rclcpp/rclcpp.hpp"

namespace flowstate_luxonis {

SpawnerNode::SpawnerNode()
    : flowstate_common::CameraSpawnerNode("luxonis_spawner", 10.0) {
  // Initial discovery is done by base class
}

std::string SpawnerNode::DeviceStateToString(XLinkDeviceState_t state) {
  switch (state) {
    case X_LINK_ANY_STATE:
      return "any_state";
    case X_LINK_BOOTED:
      return "booted";
    case X_LINK_UNBOOTED:
      return "unbooted";
    case X_LINK_BOOTLOADER:
      return "bootloader";
    case X_LINK_FLASH_BOOTED:
      return "flash_booted";
    default:
      return "unknown";
  }
}

void SpawnerNode::UpdateCameraList() {
  std::vector<dai::DeviceInfo> devices = dai::Device::getAllAvailableDevices();
  if (!devices.empty()) {
    RCLCPP_INFO(get_logger(), "Found %lu Luxonis devices", devices.size());
  }

  for (const auto& device_info : devices) {
    const std::string ip_str(device_info.name);
    const std::string serial(device_info.deviceId);
    const std::string state_str(DeviceStateToString(device_info.state));

    RCLCPP_INFO(get_logger(), "  ip: %s state: %s mxid: %s", ip_str.c_str(),
                state_str.c_str(), serial.c_str());

    RegisterDiscoveredCamera(serial, "luxonis", ip_str);

    if (IsAlreadySpawned(serial)) continue;

    RCLCPP_INFO(get_logger(), "Spawning Luxonis adapter for %s", serial.c_str());
    spawned_nodes_.push_back(std::make_unique<AdapterNode>(serial, ip_str));
  }

  // Clean up nodes that have crashed
  for (auto node_it = spawned_nodes_.begin();
       node_it != spawned_nodes_.end();) {
    if ((*node_it)->HasExitedThread()) {
      RCLCPP_INFO(get_logger(), "Camera %s has exited. Removing it.",
                  (*node_it)->GetSerial().c_str());
      node_it = spawned_nodes_.erase(node_it);
    } else {
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

std::string SpawnerNode::GetDiscoveredCameraSerial(size_t index) const {
  auto devices = dai::Device::getAllAvailableDevices();
  if (index < devices.size()) {
    return devices[index].deviceId;
  }
  return "";
}

}  // namespace flowstate_luxonis
