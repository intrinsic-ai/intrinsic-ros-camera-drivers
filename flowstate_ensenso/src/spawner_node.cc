#include "flowstate_ensenso/spawner_node.h"

#include <regex>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"

namespace flowstate_ensenso {

SpawnerNode::SpawnerNode()
    : flowstate_common::BaseSpawnerNode("ensenso_spawner", "ensenso",
                                      std::chrono::seconds(20)) {}

std::vector<std::string> SpawnerNode::GetSerials() {
  std::vector<std::string> serials;
  std::unordered_set<std::string> serial_set;
  static const std::regex topic_pattern(
      R"(^/ensenso/camera_([^/]+)/rectified/left/image$)");

  const auto topics = this->get_topic_names_and_types();
  for (const auto& topic_entry : topics) {
    std::smatch match;
    if (std::regex_match(topic_entry.first, match, topic_pattern) &&
        match.size() == 2) {
      serial_set.insert(match[1].str());
    }
  }

  serials.assign(serial_set.begin(), serial_set.end());
  return serials;
}

std::vector<std::shared_ptr<flowstate_common::BaseAdapterNode>>
SpawnerNode::SpawnNodes(const std::vector<std::string>& serials) {
  std::vector<std::shared_ptr<flowstate_common::BaseAdapterNode>> new_nodes;
  std::unordered_set<std::string> serials_to_spawn(serials.begin(), serials.end());

  for (const auto& serial : serials_to_spawn) {
    RCLCPP_INFO(get_logger(), "Spawning Ensenso adapter for camera %s",
                serial.c_str());
    new_nodes.push_back(std::make_shared<flowstate_ensenso::AdapterNode>(
        serial, std::vector<std::string>{}));
  }

  return new_nodes;
}

}  // namespace flowstate_ensenso
