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
 * Flowstate camera adapters
 *
 * This class serves as a "Factory" or "Manager" for camera adapter nodes. It
 * completely owns the logic for tracking active hardware, reporting available
 * devices to the system via the "/cameras/discover" service, and automatically
 * managing the lifecycle (creation, monitoring, and cleanup) of adapter nodes.
 *
 * Derived classes only need to act as "informants" by querying their specific
 * manufacturer SDKs and returning lists of hardware or instantiating nodes when
 * asked by this base class.
 *
 * To create a new camera spawner:
 * 1. Inherit from BaseSpawnerNode.
 * 2. Implement `GetSerials()` to query the SDK and return all connected
 *    serials.
 * 3. Implement `SpawnNodes()` to instantiate AdapterNodes for a requested list.
 * 4. Call `UpdateCameras()` from your `main()` function after instantiation
 *    to trigger the initial hardware scan.
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

  /**
   * @brief The main execution loop. It queries GetSerials(), cleans up crashed
   * or disconnected nodes, and calls SpawnNodes() for newly discovered
   * hardware. Must be called manually from main() after the object is fully
   * constructed.
   */
  void UpdateCameras() ABSL_LOCKS_EXCLUDED(nodes_mutex_);

 protected:
  /**
   * @brief Queries the manufacturer SDK for all currently connected hardware.
   * @return A vector of serial numbers for physically connected cameras.
   */
  virtual std::vector<std::string> GetSerials() = 0;

  /**
   * @brief Instantiates adapter nodes for the requested serial numbers.
   * @param serials A list of new serial numbers that need adapter nodes.
   * @return A vector of newly created BaseAdapterNode shared pointers.
   */
  virtual std::vector<std::shared_ptr<flowstate_common::BaseAdapterNode>>
  SpawnNodes(const std::vector<std::string>& serials) = 0;

  void ClearSpawnedNodes() {
    std::vector<std::shared_ptr<flowstate_common::BaseAdapterNode>>
        nodes_to_delete;
    {
      absl::MutexLock lock(&nodes_mutex_);
      nodes_to_delete = std::move(spawned_nodes_);
    }
    nodes_to_delete.clear();
  }

 private:
  // Protected mutex and vector so derived classes (like Zivid) can safely
  // access and manually shut down nodes if required by their SDK.
  mutable absl::Mutex nodes_mutex_;
  std::vector<std::shared_ptr<flowstate_common::BaseAdapterNode>> spawned_nodes_
      ABSL_GUARDED_BY(nodes_mutex_);

  bool IsAlreadySpawned(const std::string& serial) const
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(nodes_mutex_);
  void CleanupDeadNodes(const std::vector<std::string>& current_serials)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(nodes_mutex_);

  const std::string driver_type_;
  rclcpp::Service<snapshot_interfaces::srv::Discover>::SharedPtr
      discover_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace flowstate_common

#endif  // FLOWSTATE_COMMON_CAMERA_SPAWNER_NODE_H_
