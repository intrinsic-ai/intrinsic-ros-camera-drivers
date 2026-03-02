#ifndef FLOWSTATE_COMMON_CAMERA_SPAWNER_NODE_H_
#define FLOWSTATE_COMMON_CAMERA_SPAWNER_NODE_H_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/synchronization/mutex.h"
#include "flowstate_common/base_adapter_node.h"
#include "rclcpp/rclcpp.hpp"
#include "snapshot_interfaces/srv/discover.hpp"

namespace flowstate_common {

/**
 * @class BaseSpawnerNode
 * @brief Abstract base class for managing the lifecycle and discovery of
 * Flowstate camera adapters.
 *
 * This class serves as a "Factory" or "Manager" for camera adapter nodes. It
 * provides standard functionality for reporting available devices to the system
 * and automatically managing the lifecycle (creation, monitoring, and cleanup)
 * of adapter nodes.
 *
 * Key responsibilities:
 * - Provide a standard "/cameras/discover" service that lists all active
 * cameras.
 * - Maintain a thread-safe list of active camera serial numbers.
 * - Manage a list of `BaseAdapterNode` instances (the workers).
 * - Monitor adapter liveness and automatically clean up nodes that have crashed
 * or exited.
 * - Run a periodic timer to trigger camera discovery and updates.
 *
 * To create a new camera spawner:
 * 1. Inherit from BaseSpawnerNode.
 * 2. In the constructor, call the base constructor with your specific driver
 * type (e.g., "luxonis").
 * 3. Implement the pure virtual method `UpdateCameras()`.
 */
class BaseSpawnerNode : public rclcpp::Node {
 public:
  /**
   * @param node_name The name of the ROS node.
   * @param driver_type The string identifier for the driver (e.g., "luxonis",
   * "zivid").
   * @param update_period How often to run the discovery loop.
   * @param options Node options (defaults to empty).
   */
  BaseSpawnerNode(const std::string& node_name, const std::string& driver_type,
                  std::chrono::duration<double> update_period,
                  const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  virtual ~BaseSpawnerNode();

 protected:
  /**
   * @brief Pure virtual function. Derived classes must implement specific SDK
   * logic to find devices and manage the `spawned_nodes_` vector.
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
   * Should be used in UpdateCameras() after checking for new devices to also
   * clean up any dead nodes.
   */
  void CleanupExitedNodes();

  // shared_ptr because Zivid (and potentially others) require shared ownership.
  std::vector<std::shared_ptr<flowstate_common::BaseAdapterNode>>
      spawned_nodes_;

 private:
  const std::string driver_type_;
  rclcpp::Service<snapshot_interfaces::srv::Discover>::SharedPtr
      discover_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  mutable absl::Mutex discovery_mutex_;
  std::vector<std::string> discovered_serials_
      ABSL_GUARDED_BY(discovery_mutex_);
};

}  // namespace flowstate_common

#endif  // FLOWSTATE_COMMON_CAMERA_SPAWNER_NODE_H_
