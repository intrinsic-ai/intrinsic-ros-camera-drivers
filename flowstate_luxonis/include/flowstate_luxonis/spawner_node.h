#ifndef FLOWSTATE_LUXONIS_FLOWSTATE_LUXONIS_SPAWNER_NODE_H_
#define FLOWSTATE_LUXONIS_FLOWSTATE_LUXONIS_SPAWNER_NODE_H_

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "flowstate_luxonis/adapter_node.h"
#include <XLink/XLinkPublicDefines.h>

#include "rclcpp/rclcpp.hpp"
#include "snapshot_interfaces/srv/discover.hpp"

namespace flowstate_luxonis {

class SpawnerNode : public rclcpp::Node {
 public:
  SpawnerNode();

 private:
  rclcpp::Service<snapshot_interfaces::srv::Discover>::SharedPtr
      discover_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  mutable absl::Mutex serials_mutex_;
  std::vector<std::string> serials_;
  std::vector<std::unique_ptr<AdapterNode>> spawned_nodes_;

  void UpdateCameras();
  bool IsAlreadySpawned(const std::string& serial) const;
  std::string DeviceStateToString(XLinkDeviceState_t state);
};

}  // namespace flowstate_luxonis

#endif  // FLOWSTATE_LUXONIS_FLOWSTATE_LUXONIS_SPAWNER_NODE_H_