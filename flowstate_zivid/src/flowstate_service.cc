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

#include <memory>

#include "flowstate_zivid/spawner_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node_status = flowstate_zivid::SpawnerNode::Create();
  if (!node_status.ok()) {
    RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("main"),
        "Failed to create SpawnerNode: " << node_status.status());
    return EXIT_FAILURE;
  }

  std::shared_ptr<flowstate_zivid::SpawnerNode> spawner = node_status.value();

  spawner->UpdateCameras();
  rclcpp::spin(spawner);
  rclcpp::shutdown();
}
