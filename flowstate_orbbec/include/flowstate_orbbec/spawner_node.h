#ifndef FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_SPAWNER_NODE_H_
#define FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_SPAWNER_NODE_H_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "snapshot_interfaces/srv/discover.hpp"

using snapshot_interfaces::srv::Discover;

namespace flowstate_orbbec {

class SpawnerNode : public rclcpp::Node {
 public:
  SpawnerNode();

 private:
  rclcpp::Service<Discover>::SharedPtr discover_service_;
};

}  // namespace flowstate_orbbec

#endif  // FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_SPAWNER_NODE_H_
