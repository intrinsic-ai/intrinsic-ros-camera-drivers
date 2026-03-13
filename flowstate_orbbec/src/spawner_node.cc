#include "flowstate_orbbec/spawner_node.h"

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "flowstate_orbbec/adapter_node.h"
#include "orbbec_camera/ob_camera_node_driver.h"
#include "rclcpp/rclcpp.hpp"

namespace flowstate_orbbec {

SpawnerNode::SpawnerNode()
    : flowstate_common::BaseSpawnerNode("orbbec_spawner", "orbbec",
                                        std::chrono::seconds(10)) {
}

std::vector<std::string> SpawnerNode::GetSerials() {
  auto context = std::make_unique<ob::Context>();
  auto list = context->queryDeviceList();
  std::vector<std::string> serials;
  
  for (size_t i = 0; i < list->deviceCount(); i++) {
    if (std::string(list->getConnectionType(i)) == "Ethernet") {
      serials.push_back(list->serialNumber(i));
    }
  }
  return serials;
}

std::vector<std::shared_ptr<flowstate_common::BaseAdapterNode>> 
SpawnerNode::SpawnNodes(const std::vector<std::string>& serials) {
  std::vector<std::shared_ptr<flowstate_common::BaseAdapterNode>> new_nodes;
  auto context = std::make_unique<ob::Context>();
  auto list = context->queryDeviceList();
  
  std::unordered_set<std::string> serials_to_spawn(serials.begin(), serials.end());
  
  for (size_t i = 0; i < list->deviceCount(); i++) {
    std::string current_serial = list->serialNumber(i);
    
    if (serials_to_spawn.count(current_serial)) {
      std::string ip_address = list->getIpAddress(i);
      RCLCPP_INFO(get_logger(), "Spawning Orbbec node: %s at %s", 
                  current_serial.c_str(), ip_address.c_str());
                  
      new_nodes.push_back(std::make_shared<AdapterNode>(
          current_serial, std::vector<std::string>{ip_address}));
    }
  }
  return new_nodes;
}
}  // namespace flowstate_orbbec
