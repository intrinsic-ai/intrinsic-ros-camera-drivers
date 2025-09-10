#ifndef FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_SPAWNER_NODE_H_
#define FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_SPAWNER_NODE_H_

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "orbbec_camera/ob_camera_node_driver.h"
#include "rclcpp/rclcpp.hpp"
#include "snapshot_interfaces/srv/discover.hpp"

using snapshot_interfaces::srv::Discover;

namespace flowstate_orbbec {

class SpawnedNode {
 public:
  SpawnedNode(const std::string& serial, const std::string& ip_address);

  absl::Status main();

  std::string serial_;
  std::string ip_address_;
  std::unique_ptr<orbbec_camera::OBCameraNodeDriver> node_;
  std::thread thread_;
  bool exited_thread_ = false;
};

class SpawnerNode : public rclcpp::Node {
 public:
  SpawnerNode();

 private:
  rclcpp::Service<Discover>::SharedPtr discover_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  mutable absl::Mutex serials_mutex_;
  std::vector<std::string> serials_;
  std::vector<std::unique_ptr<SpawnedNode>> spawned_nodes_;

  void UpdateCameras();
  bool IsAlreadySpawned(const std::string& serial) const;
};

}  // namespace flowstate_orbbec

#endif  // FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_SPAWNER_NODE_H_
