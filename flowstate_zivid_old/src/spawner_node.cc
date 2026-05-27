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

#include "flowstate_zivid/spawner_node.h"

#include <Zivid/Application.h>
#include <Zivid/Camera.h>
#include <Zivid/Exception.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <unordered_set>
#include <vector>

#include "absl/algorithm/container.h"
#include "flowstate_zivid/adapter_node.h"
#include "snapshot_interfaces/srv/discover.hpp"
#include "zivid_camera/zivid_camera.hpp"

using ::snapshot_interfaces::srv::Describe;
using ::snapshot_interfaces::srv::Discover;

namespace flowstate_zivid {

namespace ParamNames {
constexpr auto serial_number = "serial_number";
constexpr auto frame_id = "frame_id";
constexpr auto color_space = "color_space";
constexpr auto intrinsics_source = "intrinsics_source";
}  // namespace ParamNames

absl::StatusOr<std::shared_ptr<SpawnerNode>> SpawnerNode::Create() {
  try {
    auto spawner = std::make_shared<SpawnerNode>();

    return spawner;
  } catch (const std::exception& e) {
    return absl::Status(
        absl::StatusCode::kInternal,
        absl::StrCat(
            "Failed to create SpawnerNode for flowstate Zivid camera: ",
            e.what()));
  }
}

SpawnerNode::SpawnerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("zivid_spawner", options),
      zivid_app_(std::make_shared<Zivid::Application>()) {
  RCLCPP_INFO(get_logger(), "Starting Zivid SpawnerNode...");

  using namespace std::placeholders;
  discover_service_ = create_service<Discover>(
      "/cameras/discover",
      [this](const std::shared_ptr<rmw_request_id_t>,
             const std::shared_ptr<Discover::Request>,
             const std::shared_ptr<Discover::Response> response) {
        RCLCPP_INFO(get_logger(), "=== DISCOVERY SERVICE CALLED ===");

        absl::MutexLock lock(&cameras_mutex_);

        RCLCPP_INFO(get_logger(), "Returning %d already discovered cameras",
                    static_cast<int>(discovered_camera_msgs_.size()));

        response->cameras.clear();

        for (const auto& camera : discovered_camera_msgs_) {
          response->cameras.push_back(camera);
        }

        response->success = true;
        RCLCPP_INFO(get_logger(), "=== DISCOVERY SERVICE COMPLETE ===");
      });
  timer_ = create_wall_timer(std::chrono::seconds(30),
                             [this]() { this->RefreshCameraList(); });
  RefreshCameraList();
  RCLCPP_INFO(get_logger(), "Zivid SpawnerNode is ready!");
}

SpawnerNode::~SpawnerNode() {
  ShutdownCameraNodes();
  if (timer_) {
    timer_->reset();
  }
  RCLCPP_INFO(get_logger(), "Zivid SpawnerNode shutdown complete");
}

std::vector<std::string> SpawnerNode::GetCameraNodeNames() const {
  absl::MutexLock lock(&cameras_mutex_);

  std::vector<std::string> node_names;
  for (const auto& camera : discovered_camera_msgs_) {
    node_names.push_back("zivid_" + camera.camera_id);
  }

  return node_names;
}

void SpawnerNode::RefreshCameraList() {
  absl::MutexLock lock(&cameras_mutex_);

  RCLCPP_INFO(get_logger(), "Discovering physical cameras...");
  const auto& zivid_cameras = zivid_app_->cameras();
  RCLCPP_INFO(get_logger(), "Found %zu physical camera(s)",
              zivid_cameras.size());

  std::unordered_set<std::string> discovered_serials;
  for (const auto& cam : zivid_cameras) {
    discovered_serials.insert(cam.info().serialNumber().toString());
  }

  // 1. Shut down nodes for cameras that are no longer connected.
  std::vector<std::shared_ptr<flowstate_zivid::AdapterNode>> still_active_nodes;
  for (auto& node : spawned_nodes_) {
    if (node && discovered_serials.find(node->get_serial()) ==
                    discovered_serials.end()) {
      RCLCPP_INFO(get_logger(), "Camera %s disconnected. Shutting down node.",
                  node->get_serial().c_str());
      rclcpp::shutdown(node->get_node_base_interface()->get_context());
    } else {
      still_active_nodes.push_back(node);
    }
  }
  spawned_nodes_ = still_active_nodes;

  // 2. Spawn nodes for newly discovered cameras.
  for (const std::string& serial : discovered_serials) {
    if (absl::c_any_of(spawned_nodes_, [&serial](const auto& node) {
          return node && node->get_serial() == serial;
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
  discovered_camera_msgs_.clear();
  for (const auto& node : spawned_nodes_) {
    if (!node) continue;
    snapshot_interfaces::msg::DiscoveredCamera discovered_camera;
    discovered_camera.driver_type = "zivid";
    discovered_camera.camera_id = node->get_serial();
    discovered_camera_msgs_.push_back(discovered_camera);
  }
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
