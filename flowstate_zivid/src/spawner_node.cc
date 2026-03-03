#include "flowstate_zivid/spawner_node.h"

#include <absl/algorithm/container.h>

#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "rclcpp/rclcpp.hpp"

namespace flowstate_zivid {

absl::StatusOr<std::shared_ptr<SpawnerNode>> SpawnerNode::Create() {
  try {
    return std::make_shared<SpawnerNode>();
  } catch (const std::exception& e) {
    return absl::Status(
        absl::StatusCode::kInternal,
        absl::StrCat(
            "Failed to create SpawnerNode for flowstate Zivid camera: ",
            e.what()));
  }
}

SpawnerNode::SpawnerNode(const rclcpp::NodeOptions& options)
    : flowstate_common::BaseSpawnerNode("zivid_spawner", "zivid",
                                        std::chrono::seconds(30), options),
      zivid_app_(std::make_shared<Zivid::Application>()) {
  RCLCPP_INFO(get_logger(), "Starting Zivid SpawnerNode...");
  UpdateCameras();
  RCLCPP_INFO(get_logger(), "Zivid SpawnerNode is ready!");
}

SpawnerNode::~SpawnerNode() { ShutdownCameraNodes(); }

std::vector<std::string> SpawnerNode::GetSerials() {
  std::vector<std::string> serials;
  for (const auto& cam : zivid_app_->cameras()) {
    serials.push_back(cam.info().serialNumber().toString());
  }
  return serials;
}

std::vector<std::shared_ptr<flowstate_common::BaseAdapterNode>>
SpawnerNode::SpawnNodes(const std::vector<std::string>& serials) {
  std::vector<std::shared_ptr<flowstate_common::BaseAdapterNode>> new_nodes;

  for (const auto& serial : serials) {
    RCLCPP_INFO(get_logger(), "Spawning Zivid node: %s", serial.c_str());
    try {
      std::vector<rclcpp::Parameter> parameters;
      parameters.emplace_back("serial_number", serial);

      rclcpp::NodeOptions node_options;
      node_options.parameter_overrides(parameters);

      new_nodes.push_back(std::make_shared<flowstate_zivid::AdapterNode>(
          serial, node_options, zivid_app_));
    } catch (const std::exception& e) {
      RCLCPP_ERROR_STREAM(
          get_logger(), "Failed to create AdapterNode for zivid: " << e.what());
    }
  }
  return new_nodes;
}

void SpawnerNode::ShutdownCameraNodes() {
  if (spawned_nodes_.empty()) {
    return;
  }
  RCLCPP_INFO(get_logger(), "Shutting down %zu camera node(s)...",
              spawned_nodes_.size());

  // Request nodes to shut down.
  for (auto& node : spawned_nodes_) {
    // The node might be null if creation failed but was still added to the
    // list.
    if (node) {
      rclcpp::shutdown(node->get_node_base_interface()->get_context());
    }
  }

  spawned_nodes_.clear();
}
}  // namespace flowstate_zivid
