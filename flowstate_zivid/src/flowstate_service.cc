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
