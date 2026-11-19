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

#include "flowstate_realsense/spawner_node.h"

#include <algorithm>
#include <chrono>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/container/flat_hash_set.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "flowstate_realsense/adapter_node.h"
#include "librealsense2/rs.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/types.h"
#include "snapshot_interfaces/msg/discovered_camera.hpp"
#include "snapshot_interfaces/srv/discover.hpp"

namespace flowstate_realsense {
namespace {

constexpr absl::string_view kDriverType = "realsense";

// Sleeps for `duration` in 100 ms steps so SIGTERM ends waits promptly.
// Returns false if rclcpp::ok() becomes false before `duration` elapses.
bool SleepWhileOk(std::chrono::milliseconds duration) {
  static constexpr std::chrono::milliseconds kStep(100);
  for (std::chrono::milliseconds elapsed(0); elapsed < duration;
       elapsed += kStep) {
    if (!rclcpp::ok()) return false;
    std::this_thread::sleep_for(std::min(kStep, duration - elapsed));
  }
  return rclcpp::ok();
}

}  // namespace

SpawnerNode::SpawnerNode(std::vector<std::string> serials)
    : rclcpp::Node("realsense_spawner"),
      configured_serials_(std::move(serials)) {
  if (!configured_serials_.empty()) {
    RCLCPP_INFO(get_logger(), "Serving only the RealSense cameras %s",
                absl::StrJoin(configured_serials_, ", ").c_str());
  }
  using snapshot_interfaces::srv::Discover;
  discover_service_ = create_service<Discover>(
      "/cameras/discover",
      [this](std::shared_ptr<rmw_request_id_t>,
             std::shared_ptr<Discover::Request>,
             std::shared_ptr<Discover::Response> response) {
        RCLCPP_INFO_ONCE(get_logger(),
                         "Discover service called (logging once)");
        absl::MutexLock lock(&nodes_mutex_);
        for (const std::shared_ptr<AdapterNode>& node : spawned_nodes_) {
          snapshot_interfaces::msg::DiscoveredCamera camera;
          camera.driver_type = std::string(kDriverType);
          camera.camera_id = node->GetSerial();
          response->cameras.push_back(std::move(camera));
        }
        response->success = true;
      });
  timer_ = create_wall_timer(std::chrono::seconds(10),
                             [this]() { UpdateCameras(); });
}

SpawnerNode::~SpawnerNode() {
  timer_.reset();
  discover_service_.reset();
  std::vector<std::shared_ptr<AdapterNode>> nodes_to_destroy;
  {
    absl::MutexLock lock(&nodes_mutex_);
    nodes_to_destroy = std::move(spawned_nodes_);
  }
  nodes_to_destroy.clear();
  RCLCPP_INFO(get_logger(), "Camera SpawnerNode shutdown complete");
}

std::optional<std::vector<std::string>> SpawnerNode::QuerySerials() {
  std::vector<std::string> serials;
  absl::flat_hash_set<std::string> seen;
  try {
    for (const rs2::device& device : context_.query_devices()) {
      if (!device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) continue;
      std::string serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
      if (!configured_serials_.empty() &&
          absl::c_find(configured_serials_, serial) ==
              configured_serials_.end()) {
        continue;
      }
      if (seen.insert(serial).second) {
        serials.push_back(std::move(serial));
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "librealsense device query failed: %s",
                 e.what());
    return std::nullopt;
  }
  return serials;
}

void SpawnerNode::UpdateCameras() {
  const std::optional<std::vector<std::string>> current_serials =
      QuerySerials();
  if (!rclcpp::ok()) return;

  std::vector<std::shared_ptr<AdapterNode>> removed;
  {
    absl::MutexLock lock(&nodes_mutex_);
    for (auto it = spawned_nodes_.begin(); it != spawned_nodes_.end();) {
      const std::string node_serial = (*it)->GetSerial();
      const bool has_exited = (*it)->HasExitedThread();
      const bool is_unplugged =
          current_serials.has_value() &&
          absl::c_find(*current_serials, node_serial) == current_serials->end();
      if (has_exited || is_unplugged) {
        if (has_exited) {
          RCLCPP_INFO(get_logger(), "Camera %s thread exited. Removing node.",
                      node_serial.c_str());
        } else {
          RCLCPP_WARN(get_logger(),
                      "Camera %s physically disconnected. Removing node.",
                      node_serial.c_str());
        }
        removed.push_back(std::move(*it));
        it = spawned_nodes_.erase(it);
      } else {
        ++it;
      }
    }
  }

  for (std::shared_ptr<AdapterNode>& node : removed) {
    node.reset();
    RCLCPP_INFO(get_logger(), "Waiting a few seconds after deleting node");
    if (!SleepWhileOk(std::chrono::milliseconds(2000))) return;
    RCLCPP_INFO(get_logger(), "Done waiting after deleting node");
  }
  if (!rclcpp::ok() || !current_serials.has_value()) return;

  std::vector<std::string> serials_to_spawn;
  {
    absl::MutexLock lock(&nodes_mutex_);
    for (const std::string& serial : *current_serials) {
      const bool already_spawned = absl::c_any_of(
          spawned_nodes_, [&serial](const std::shared_ptr<AdapterNode>& node) {
            return node->GetSerial() == serial;
          });
      if (!already_spawned) {
        serials_to_spawn.push_back(serial);
      }
    }
  }

  for (const std::string& serial : serials_to_spawn) {
    if (!rclcpp::ok()) return;
    RCLCPP_INFO(get_logger(), "Spawning RealSense node for serial %s",
                serial.c_str());
    try {
      std::shared_ptr<AdapterNode> node = std::make_shared<AdapterNode>(serial);
      absl::MutexLock lock(&nodes_mutex_);
      spawned_nodes_.push_back(std::move(node));
    } catch (const std::exception& e) {
      if (rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Could not create adapter for camera %s: %s",
                     serial.c_str(), e.what());
      }
    }
  }
}

}  // namespace flowstate_realsense
