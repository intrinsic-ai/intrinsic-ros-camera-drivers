#include "flowstate_ensenso/spawner_node.h"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<flowstate_ensenso::SpawnerNode>();
  node->UpdateCameras();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
