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

#include <cstdlib>
#include <exception>
#include <memory>

#include "flowstate_realsense/spawner_node.h"
#include "rclcpp/rclcpp.hpp"

// The spawner outside Flowstate, for local testing: it reads no service config
// and serves every RealSense camera on the host. It serves /cameras/discover;
// each camera's adapter runs its own executor thread. The Flowstate service
// runs flowstate_service instead.
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    std::shared_ptr<flowstate_realsense::SpawnerNode> node =
        std::make_shared<flowstate_realsense::SpawnerNode>();
    if (rclcpp::ok()) {
      node->UpdateCameras();
    }
    if (rclcpp::ok()) {
      RCLCPP_INFO(node->get_logger(), "Spinning spawner... Ctrl+C to exit");
      rclcpp::spin(node);
    }
  } catch (const std::exception& e) {
    // Exceptions raised while rclcpp is already shutting down (e.g., SIGTERM
    // interrupting spin/node creation) are expected and exit cleanly.
    if (rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("realsense_spawner"), "Fatal error: %s",
                   e.what());
      rclcpp::shutdown();
      return EXIT_FAILURE;
    }
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
