/*
 * Copyright 2026 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef FLOWSTATE_ZIVID_FLOWSTATE_ZIVID_SPAWNER_NODE_H_
#define FLOWSTATE_ZIVID_FLOWSTATE_ZIVID_SPAWNER_NODE_H_

#include <Zivid/Application.h>
#include <absl/status/statusor.h>

#include <memory>
#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "flowstate_common/base_spawner_node.h"
#include "flowstate_zivid/adapter_node.h"
#include "rclcpp/rclcpp.hpp"

namespace flowstate_zivid {
/**
 * @class SpawnerNode
 * @brief A ROS 2 node that discovers cameras and manages adapter nodes for
 * zivid_camera::ZividCamera node from the official Zivid ROS driver
 * (https://github.com/intrinsic-dev/zivid-ros/blob/flowstate/zivid_camera/include/zivid_camera/zivid_camera.hpp).
 *
 * -----
 * Key Responsibilities:
 * 1.  Camera Discovery:
 *     On startup, it automatically discovers all physically connected Zivid
 *     cameras.
 * 2.  Node Spawning:
 *     For each discovered camera, it generates a AdapterNode.
 *     Each AdapterNode is a separate ROS 2 node that directly
 *     interacts with a single zivid_camera::ZividCamera ROS2 node from the
 *     official Zivid ROS driver, translating Flowstate sdk-ros ROS2 service
 *     calls into Zivid ROS2 driver service calls and publishing required
 *     information for Flowstate services.
 * 3.  Discovery Service:
 *     It provides a ROS service ("/cameras/discover") for
 *     Flowstate that allows to query for a list of available cameras with their
 *     serial numbers.
 */
class SpawnerNode : public flowstate_common::BaseSpawnerNode {
 public:
  SpawnerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  ~SpawnerNode() override;

  static absl::StatusOr<std::shared_ptr<SpawnerNode>> Create();

 protected:
  std::vector<std::string> GetSerials() override;
  std::vector<std::shared_ptr<flowstate_common::BaseAdapterNode>> SpawnNodes(
      const std::vector<std::string>& serials) override;

 private:
  std::shared_ptr<Zivid::Application> zivid_app_;
};
}  // namespace flowstate_zivid

#endif  // FLOWSTATE_ZIVID_FLOWSTATE_ZIVID_SPAWNER_NODE_H_
