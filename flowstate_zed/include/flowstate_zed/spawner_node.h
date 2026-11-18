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

#ifndef FLOWSTATE_ZED_SPAWNER_NODE_H_
#define FLOWSTATE_ZED_SPAWNER_NODE_H_

#include <memory>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/synchronization/mutex.h"
#include "flowstate_zed/adapter_node.h"
#include "flowstate_zed/camera_discovery.h"
#include "rclcpp/rclcpp.hpp"
#include "snapshot_interfaces/srv/discover.hpp"

namespace flowstate_zed {

// Creates one AdapterNode per ZED X camera and serves /cameras/discover with
// per-model driver_type values. Cameras are enumerated once, in the
// constructor: GMSL cameras do not hotplug, and GMSL discovery can disturb a
// running capture, so the periodic UpdateCameras() pass does not touch the SDK.
class SpawnerNode : public rclcpp::Node {
 public:
  SpawnerNode();
  ~SpawnerNode() override;

  // Removes adapters whose threads have exited and spawns adapters for any
  // discovered cameras that are not currently running.
  void UpdateCameras() ABSL_LOCKS_EXCLUDED(nodes_mutex_);

 private:
  const std::vector<DiscoveredCamera> cameras_;
  absl::Mutex nodes_mutex_;
  std::vector<std::shared_ptr<AdapterNode>> spawned_nodes_
      ABSL_GUARDED_BY(nodes_mutex_);
  rclcpp::Service<snapshot_interfaces::srv::Discover>::SharedPtr
      discover_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace flowstate_zed

#endif  // FLOWSTATE_ZED_SPAWNER_NODE_H_
