// Copyright 2026 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "flowstate_orbbec/spawner_node.h"

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "flowstate_orbbec/adapter_node.h"
#include "orbbec_camera/ob_camera_node_driver.h"
#include "rclcpp/rclcpp.hpp"

namespace flowstate_orbbec {

ABSL_CONST_INIT absl::Mutex SpawnerNode::s_discovery_mutex(absl::kConstInit);

SpawnerNode::SpawnerNode()
    : flowstate_common::BaseSpawnerNode("orbbec_spawner", "orbbec",
                                        std::chrono::seconds(10)),
      context_(std::make_shared<ob::Context>()) {}

std::vector<std::string> SpawnerNode::GetSerials() {
  absl::MutexLock lock(&s_discovery_mutex);
  auto list = context_->queryDeviceList();
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
  RCLCPP_INFO(get_logger(), "flowstate_orbbec::SpawnerNode::SpawnNodes()");
  std::vector<std::shared_ptr<flowstate_common::BaseAdapterNode>> new_nodes;
  std::shared_ptr<ob::DeviceList> list;
  {
    absl::MutexLock lock(&s_discovery_mutex);
    list = context_->queryDeviceList();
  }
  
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
