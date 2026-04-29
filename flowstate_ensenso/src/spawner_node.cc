#include "flowstate_ensenso/spawner_node.h"
#include "flowstate_ensenso/adapter_node.h"

#include <unordered_set>
#include <nxLib.h>

#include "rclcpp/rclcpp.hpp"

namespace flowstate_ensenso {

SpawnerNode::SpawnerNode()
    : flowstate_common::BaseSpawnerNode("ensenso_spawner", "ensenso",
                                      std::chrono::seconds(20)) {
  nxLibInitialize();
}

SpawnerNode::~SpawnerNode() {
  nxLibFinalize();
}

std::vector<std::string> SpawnerNode::GetSerials() {
  std::vector<std::string> serials;
  try {
    NxLibItem cameras = NxLibItem()["Cameras"]["BySerialNo"];
    
    if (cameras.exists()) {
      for (int i = 0; i < cameras.count(); ++i) {
        std::string name = cameras[i].name();
        // NXLib API automatically places these in the BySerialNo tree, we want to ignore those keys.
        if (name != "BySerialNo" && name != "ByEepromId") {
          serials.push_back(name);
        }
      }
    }

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to get serials from NxLib: %s", e.what());
  }
  return serials;
}

std::vector<std::shared_ptr<flowstate_common::BaseAdapterNode>>
SpawnerNode::SpawnNodes(const std::vector<std::string>& serials) {
  std::vector<std::shared_ptr<flowstate_common::BaseAdapterNode>> new_nodes;
  std::unordered_set<std::string> serials_to_spawn(serials.begin(), serials.end());

  for (const auto& serial : serials_to_spawn) {
    bool is_available = false;
    
    // Check if the camera is ready to be claimed before spawning
    try {
      NxLibItem available_item = NxLibItem()["Cameras"]["BySerialNo"][serial]["Status"]["Available"];
      if (available_item.exists()) {
        is_available = available_item.asBool();
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "NxLib error checking availability for %s: %s", serial.c_str(), e.what());
    }

    if (is_available) {
      RCLCPP_INFO(get_logger(), "Spawning Ensenso adapter for camera %s", serial.c_str());
      new_nodes.push_back(std::make_shared<flowstate_ensenso::AdapterNode>(serial, std::vector<std::string>{}));
    } else {
      RCLCPP_WARN(get_logger(), "Camera %s physically present but UNAVAILABLE. Waiting...", serial.c_str());
    }
  }

  return new_nodes;
}

}  // namespace flowstate_ensenso
