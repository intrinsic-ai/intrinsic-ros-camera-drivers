#include <memory>

#include "flowstate_orbbec/spawner_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<flowstate_orbbec::SpawnerNode> node =
      std::make_shared<flowstate_orbbec::SpawnerNode>();
  node->UpdateCameras();

  rclcpp::spin(node);
  rclcpp::shutdown();
}
