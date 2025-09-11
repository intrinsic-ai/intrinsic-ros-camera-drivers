#ifndef FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_ADAPTER_NODE_H_
#define FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_ADAPTER_NODE_H_

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "orbbec_camera/ob_camera_node_driver.h"
#include "rclcpp/rclcpp.hpp"
#include "snapshot_interfaces/srv/describe.hpp"
#include "snapshot_interfaces/srv/snapshot.hpp"

namespace flowstate_orbbec {

class AdapterNode : public rclcpp::Node {
 public:
  AdapterNode(const std::string& serial, const std::string& ip_address);

  absl::Status main();

  std::string serial_;
  std::string ip_address_;
  std::unique_ptr<orbbec_camera::OBCameraNodeDriver> orbbec_node_;
  std::thread thread_;
  bool exited_thread_ = false;
  mutable absl::Mutex mutex_;

  rclcpp::Service<snapshot_interfaces::srv::Describe>::SharedPtr
      describe_service_;
  rclcpp::Service<snapshot_interfaces::srv::Snapshot>::SharedPtr
      snapshot_service_;
};

}  // namespace flowstate_orbbec

#endif  // FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_ADAPTER_NODE_H_
