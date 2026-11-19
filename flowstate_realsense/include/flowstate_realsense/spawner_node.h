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

#ifndef FLOWSTATE_REALSENSE_SPAWNER_NODE_H_
#define FLOWSTATE_REALSENSE_SPAWNER_NODE_H_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/synchronization/mutex.h"
#include "flowstate_realsense/adapter_node.h"
#include "librealsense2/rs.hpp"
#include "rclcpp/rclcpp.hpp"
#include "snapshot_interfaces/srv/discover.hpp"

namespace flowstate_realsense {

// Discovers the RealSense cameras on the host's USB buses (librealsense's
// device list) and spawns one AdapterNode per serial. The driver type is
// "realsense": Flowstate addresses a camera's services as
// /realsense_<serial>/{describe,snapshot}. Given serials (the service
// config's), it serves only those cameras; empty serves every camera found.
class SpawnerNode : public rclcpp::Node {
 public:
  explicit SpawnerNode(std::vector<std::string> serials = {});
  ~SpawnerNode() override;

  // Cleans up exited or disconnected adapters and spawns adapters for newly
  // discovered cameras.
  void UpdateCameras() ABSL_LOCKS_EXCLUDED(nodes_mutex_);

 private:
  // Queries librealsense for connected serials matching configured_serials_.
  // Returns std::nullopt on a transient SDK query error so live adapters are
  // not falsely torn down as unplugged.
  std::optional<std::vector<std::string>> QuerySerials();

  rs2::context context_;
  const std::vector<std::string> configured_serials_;
  absl::Mutex nodes_mutex_;
  std::vector<std::shared_ptr<AdapterNode>> spawned_nodes_
      ABSL_GUARDED_BY(nodes_mutex_);
  rclcpp::Service<snapshot_interfaces::srv::Discover>::SharedPtr
      discover_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace flowstate_realsense

#endif  // FLOWSTATE_REALSENSE_SPAWNER_NODE_H_
