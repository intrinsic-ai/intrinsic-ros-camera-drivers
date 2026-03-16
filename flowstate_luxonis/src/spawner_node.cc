#include "flowstate_luxonis/spawner_node.h"

#include <XLink/XLinkPublicDefines.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "depthai/device/Device.hpp"
#include "flowstate_luxonis/adapter_node.h"
#include "rclcpp/rclcpp.hpp"

namespace flowstate_luxonis {

SpawnerNode::SpawnerNode()
    : flowstate_common::BaseSpawnerNode("luxonis_spawner", "luxonis",
                                        std::chrono::seconds(20)) {}

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

std::vector<std::string> SpawnerNode::GetSerials() {
  std::vector<std::string> serials;
  std::vector<dai::DeviceInfo> devices = dai::Device::getAllConnectedDevices();
  for (const auto& device_info : devices) {
    serials.push_back(device_info.deviceId);
  }
  return serials;
}

std::vector<std::shared_ptr<flowstate_common::BaseAdapterNode>>
SpawnerNode::SpawnNodes(const std::vector<std::string>& serials) {
  std::vector<std::shared_ptr<flowstate_common::BaseAdapterNode>> new_nodes;
  std::vector<dai::DeviceInfo> devices = dai::Device::getAllAvailableDevices();

  std::unordered_set<std::string> serials_to_spawn(serials.begin(),
                                                   serials.end());

  for (const auto& device_info : devices) {
    if (serials_to_spawn.count(device_info.deviceId)) {
      std::string ip_str = device_info.name;
      RCLCPP_INFO(get_logger(), "Spawning Luxonis node: %s at %s",
                  device_info.deviceId.c_str(), ip_str.c_str());

      new_nodes.push_back(std::make_shared<AdapterNode>(
          device_info.deviceId, std::vector<std::string>{ip_str}));
    }
  }
  return new_nodes;
}

}  // namespace flowstate_luxonis
