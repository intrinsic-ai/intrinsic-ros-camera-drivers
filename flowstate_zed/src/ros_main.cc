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

#include <exception>
#include <memory>

#include "flowstate_zed/spawner_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    std::shared_ptr<flowstate_zed::SpawnerNode> node =
        std::make_shared<flowstate_zed::SpawnerNode>();
    if (rclcpp::ok()) {
      node->UpdateCameras();
    }
    if (rclcpp::ok()) {
      rclcpp::spin(node);
    }
  } catch (const std::exception& e) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("zed_spawner"), "Fatal error: %s",
                   e.what());
      rclcpp::shutdown();
      return 1;
    }
  }
  rclcpp::shutdown();
  return 0;
}
