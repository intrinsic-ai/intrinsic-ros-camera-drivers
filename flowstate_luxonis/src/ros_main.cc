#include <memory>

#include "flowstate_luxonis/spawner_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<flowstate_luxonis::SpawnerNode> node =
      std::make_shared<flowstate_luxonis::SpawnerNode>();
  node->UpdateCameras();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
