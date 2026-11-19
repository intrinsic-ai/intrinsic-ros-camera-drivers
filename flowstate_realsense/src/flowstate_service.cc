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
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "flowstate/realsense_driver_config.pb.h"
#include "flowstate_realsense/spawner_node.h"
#include "intrinsic/resources/proto/runtime_context.pb.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  intrinsic_proto::config::RuntimeContext runtime_context;
  std::ifstream runtime_context_file("/etc/intrinsic/runtime_config.pb",
                                     std::ios::binary);
  if (!runtime_context.ParseFromIstream(&runtime_context_file)) {
    std::cerr << "Unable to parse runtime context file\n";
    return EXIT_FAILURE;
  }

  realsense_driver::RealSenseDriverConfig config;
  if (!runtime_context.config().UnpackTo(&config)) {
    std::cerr << "Unable to parse config proto\n";
    return EXIT_FAILURE;
  }

  rclcpp::init(argc, argv);
  try {
    std::shared_ptr<flowstate_realsense::SpawnerNode> spawner_node =
        std::make_shared<flowstate_realsense::SpawnerNode>(
            std::vector<std::string>(config.serials().begin(),
                                     config.serials().end()));
    if (rclcpp::ok()) {
      spawner_node->UpdateCameras();
    }
    if (rclcpp::ok()) {
      RCLCPP_INFO(spawner_node->get_logger(),
                  "Spinning spawner... Ctrl+C to exit");
      rclcpp::spin(spawner_node);
    }
    RCLCPP_INFO(spawner_node->get_logger(), "Shutting down");
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
