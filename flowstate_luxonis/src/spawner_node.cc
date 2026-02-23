#include "flowstate_luxonis/spawner_node.h"

#include "depthai/device/Device.hpp"

namespace flowstate_luxonis {

SpawnerNode::SpawnerNode()
    : flowstate_common::BaseSpawnerNode("luxonis_spawner", "luxonis",
                                        std::chrono::seconds(10)) {
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

  std::vector<std::string> current_serials;
  for (const auto& device_info : devices) {
    const std::string ip_str(device_info.name);
    const std::string serial(device_info.deviceId);
    const std::string state_str(DeviceStateToString(device_info.state));

    RCLCPP_INFO(get_logger(), "  ip: %s state: %s mxid: %s", ip_str.c_str(),
                state_str.c_str(), serial.c_str());
    current_serials.push_back(serial);
    if (IsAlreadySpawned(serial)) continue;
    RCLCPP_INFO(get_logger(), "Spawning it...");
    spawned_nodes_.push_back(std::make_shared<AdapterNode>(serial, ip_str));
  }

  // See if any camera nodes have crashed. If so, close them so we can respawn
  CleanupExitedNodes();

  // Update the Base Class with the list of active serials
  SetDiscoveredSerials(current_serials);
}

}  // namespace flowstate_luxonis