#include "flowstate_zivid/spawner_node.h"
#include <absl/algorithm/container.h>
#include "absl/strings/str_cat.h"

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
    : flowstate_common::BaseSpawnerNode("zivid_spawner", "zivid", std::chrono::seconds(30), options),
      zivid_app_(std::make_shared<Zivid::Application>()) {
  RCLCPP_INFO(get_logger(), "Starting Zivid SpawnerNode...");
  UpdateCameras();
  RCLCPP_INFO(get_logger(), "Zivid SpawnerNode is ready!");
}

std::vector<std::string> SpawnerNode::GetCameraNodeNames() const {
    std::vector<std::string> names;
  for (const auto& node : spawned_nodes_) {
    if(node) {
      names.push_back("zivid_" + node->GetSerial());
    }
  }
  return names;
}

void SpawnerNode::UpdateCameras() {
  RCLCPP_INFO(get_logger(), "Discovering physical cameras...");
  const auto& zivid_cameras = zivid_app_->cameras();
  RCLCPP_INFO(get_logger(), "Found %zu physical camera(s)",
              zivid_cameras.size());

  std::unordered_set<std::string> discovered_serials;
  std::vector<std::string> current_serials;

  for (const auto& cam : zivid_cameras) {
    std::string serial = cam.info().serialNumber().toString();
    discovered_serials.insert(serial);
    current_serials.push_back(serial);
  }

  // 1. Shut down nodes for cameras that are no longer connected.
std::vector<std::shared_ptr<flowstate_common::CameraAdapterNode>> still_active_nodes;
  for (auto& node : spawned_nodes_) {
    if (node && discovered_serials.find(node->GetSerial()) ==
                    discovered_serials.end()) {
      RCLCPP_INFO(get_logger(), "Camera %s disconnected. Shutting down node.",
                  node->GetSerial().c_str());
      rclcpp::shutdown(node->get_node_base_interface()->get_context());
    } else {
      still_active_nodes.push_back(node);
    }
  }
  spawned_nodes_ = still_active_nodes;

  // 2. Spawn nodes for newly discovered cameras.
  for (const std::string& serial : discovered_serials) {
    if (absl::c_any_of(spawned_nodes_, [&serial](const auto& node) {
          return node && node->GetSerial() == serial;
        })) {
      continue;
    }
    RCLCPP_INFO(get_logger(), "New camera found: %s. Spawning node.",
                serial.c_str());
    try {
      std::vector<rclcpp::Parameter> parameters;
      parameters.emplace_back("serial_number", serial);

      rclcpp::NodeOptions node_options;
      node_options.parameter_overrides(parameters);

      auto camera_node = std::make_shared<flowstate_zivid::AdapterNode>(
          serial, node_options, zivid_app_);
      spawned_nodes_.push_back(camera_node);
      RCLCPP_INFO(get_logger(), "Created AdapterNode for zivid camera %s",
                  serial.c_str());
    } catch (const std::exception& e) {
      RCLCPP_ERROR_STREAM(get_logger(),
                          "Failed to create AdapterNode for zivid camera "
                              << serial << ": " << e.what());
    }
  }

  // 3. Update the list for the discovery service.
  SetDiscoveredSerials(current_serials);
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
