#ifndef FLOWSTATE_COMMON_CAMERA_SPAWNER_NODE_H_
#define FLOWSTATE_COMMON_CAMERA_SPAWNER_NODE_H_

#include <string>
#include <vector>
#include <chrono>

#include "absl/synchronization/mutex.h"
#include "rclcpp/rclcpp.hpp"
#include "snapshot_interfaces/srv/discover.hpp"
#include "flowstate_common/camera_adapter_node.h"

namespace flowstate_common {

class CameraSpawnerNode : public rclcpp::Node {
 public:
  /**
   * @param node_name The name of the ROS node.
   * @param driver_type The string identifier for the driver (e.g., "luxonis", "zivid").
   * @param update_period How often to run the discovery loop.
   * @param options Node options (defaults to empty).
   */
  CameraSpawnerNode(const std::string& node_name,
                  const std::string& driver_type,
                  std::chrono::duration<double> update_period,
                  const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  virtual ~CameraSpawnerNode() = default;

 protected:
  /**
   * @brief Pure virtual function. Derived classes must implement specific SDK logic
   * to find devices and manage the `spawned_nodes_` vector.
   */
  virtual void UpdateCameras() = 0;

  /**
   * @brief Derived classes must call this at the end of UpdateCameras()
   * to update the list available to the Discover service.
   */
  void SetDiscoveredSerials(const std::vector<std::string>& serials);

  /**
   * @brief Checks if a node with this serial already exists in spawned_nodes_.
   */
  bool IsAlreadySpawned(const std::string& serial) const;

  /**
   * @brief Iterates through spawned_nodes_ and removes any that have exited.
   */
  void CleanupExitedNodes();

  // We use shared_ptr because Zivid (and potentially others) require shared ownership.
  std::vector<std::shared_ptr<flowstate_common::CameraAdapterNode>> spawned_nodes_;

 private:
  const std::string driver_type_;
  rclcpp::Service<snapshot_interfaces::srv::Discover>::SharedPtr discover_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  mutable absl::Mutex discovery_mutex_;
  std::vector<std::string> discovered_serials_ ABSL_GUARDED_BY(discovery_mutex_);
};

}  // namespace flowstate_common

#endif  // FLOWSTATE_COMMON_CAMERA_SPAWNER_NODE_H_