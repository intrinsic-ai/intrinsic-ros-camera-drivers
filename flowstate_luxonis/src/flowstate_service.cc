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

#include <fstream>
#include <string>

#include "flowstate/luxonis_driver_config.pb.h"
#include "flowstate_luxonis/spawner_node.h"
#include "intrinsic/resources/proto/runtime_context.pb.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  intrinsic_proto::config::RuntimeContext runtime_context;
  std::ifstream runtime_context_file;
  runtime_context_file.open("/etc/intrinsic/runtime_config.pb",
                            std::ios::binary);
  if (!runtime_context.ParseFromIstream(&runtime_context_file)) {
    std::cerr << "Unable to parse runtime context file" << std::endl;
    return EXIT_FAILURE;
  }
  luxonis_driver::LuxonisDriverConfig config;
  if (!runtime_context.config().UnpackTo(&config)) {
    std::cerr << "Unable to parse config proto" << std::endl;
    return EXIT_FAILURE;
  }

  rclcpp::init(argc, argv);

  std::cout << "Creating spawner node..." << std::endl;
  std::shared_ptr<flowstate_luxonis::SpawnerNode> spawner_node =
      std::make_shared<flowstate_luxonis::SpawnerNode>();
  spawner_node->UpdateCameras();

  // In the future, we should set the ip_address parameter with what we
  // find in the context file config proto.

  RCLCPP_INFO(spawner_node->get_logger(), "Spinning spawner... Ctrl+C to exit");
  rclcpp::spin(spawner_node);

  RCLCPP_INFO(spawner_node->get_logger(), "Shutting down");
  rclcpp::shutdown();
  spawner_node.reset();
  return EXIT_SUCCESS;
}
