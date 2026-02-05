#include "flowstate_orbbec/spawner_node.h"

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace flowstate_orbbec {

SpawnerNode::SpawnerNode()
    : flowstate_common::CameraSpawnerNode("orbbec_spawner", 10.0) {
  // Initial discovery is done by base class
}

void SpawnerNode::UpdateCameraList() {
  auto context = std::make_unique<ob::Context>();
  auto list = context->queryDeviceList();

  for (size_t i = 0; i < list->deviceCount(); i++) {
    if (std::string(list->getConnectionType(i)) != std::string("Ethernet")) {
      continue;
    }
    std::string serial = list->serialNumber(i);
    std::string ip_address = list->getIpAddress(i);

    RCLCPP_INFO(get_logger(), "Found Orbbec device: %s at %s", serial.c_str(),
                ip_address.c_str());

    RegisterDiscoveredCamera(serial, "orbbec", ip_address);

    if (IsAlreadySpawned(serial)) continue;

    RCLCPP_INFO(get_logger(), "Spawning Orbbec adapter for %s", serial.c_str());
    spawned_nodes_.push_back(std::make_unique<AdapterNode>(serial, ip_address));
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
  auto context = std::make_unique<ob::Context>();
  auto list = context->queryDeviceList();
  if (index < list->deviceCount()) {
    return list->serialNumber(index);
  }
  return "";
}

std::string SpawnerNode::GetDiscoveredCameraIp(size_t index) const {
  auto context = std::make_unique<ob::Context>();
  auto list = context->queryDeviceList();
  if (index < list->deviceCount()) {
    return list->getIpAddress(index);
  }
  return "";
}

}  // namespace flowstate_orbbec
