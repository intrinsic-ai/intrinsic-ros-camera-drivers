#include <memory>

#include "flowstate_zivid/spawner_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = flowstate_zivid::SpawnerNode::Create();
  rclcpp::spin(*node);
  rclcpp::shutdown();
}
