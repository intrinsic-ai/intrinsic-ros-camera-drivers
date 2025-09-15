#if 0
#include <memory>

#include "flowstate_orbbec/spawner_node.h"
#include "rclcpp/rclcpp.hpp"
#endif

#include "depthai/depthai.hpp"

int main(int argc, char **argv)
{
#if 0
  rclcpp::init(argc, argv);
  std::shared_ptr<flowstate_orbbec::SpawnerNode> node =
      std::make_shared<flowstate_orbbec::SpawnerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
#endif
  return 0;
}
