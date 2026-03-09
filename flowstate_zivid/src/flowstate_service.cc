#include <memory>

#include "flowstate_zivid/spawner_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = flowstate_zivid::SpawnerNode::Create();
  if (!node.ok()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("main"),
                        "Failed to create SpawnerNode: " << node.status());
    return EXIT_FAILURE;
  }
  node->UpdateCameras();
  rclcpp::spin(*node);
  rclcpp::shutdown();
}
