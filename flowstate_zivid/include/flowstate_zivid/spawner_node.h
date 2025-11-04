#ifndef FLOWSTATE_ZIVID_FLOWSTATE_ZIVID_SPAWNER_NODE_H_
#define FLOWSTATE_ZIVID_FLOWSTATE_ZIVID_SPAWNER_NODE_H_

#include <absl/status/status.h>
#include <absl/status/statusor.h>
#include <absl/strings/str_cat.h>
#include <absl/synchronization/mutex.h>

#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "adapter_node.h"
#include "snapshot_interfaces/msg/discovered_camera.hpp"
#include "snapshot_interfaces/srv/discover.hpp"
#include "zivid_camera/zivid_camera.hpp"

namespace flowstate_zivid {
/**
 * @class SpawnerNode
 * @brief A ROS 2 node that discovers cameras and manages adapter nodes for
 * zivid_camera::ZividCamera node from the official Zivid ROS driver
 * (https://github.com/intrinsic-dev/zivid-ros/blob/flowstate/zivid_camera/include/zivid_camera/zivid_camera.hpp).
 *
 * -----
 * Key Responsibilities:
 * 1.  Camera Discovery:
 *     On startup, it automatically discovers all physically connected Zivid
 *     cameras. It also supports using a "file camera" for testing and
 *     development by reading from a ".zfc" file.
 * 2.  Node Spawning:
 *     For each discovered camera, it generates a AdapterNode.
 *     Each AdapterNode is a separate ROS 2 node that directly
 *     interacts with a single zivid_camera::ZividCamera ROS2 node from the
 *     official Zivid ROS driver, translating Flowstate sdk-ros ROS2 service
 *     calls into Zivid ROS2 driver service calls and publishing required
 *     information for Flowstate services.
 * 3.  Discovery Service:
 *     It provides a ROS service ("/cameras/discover") for
 *     Flowstate that allows to query for a list of available cameras with their
 *     serial numbers.
 */
class SpawnerNode : public rclcpp::Node {
 public:
  SpawnerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  static absl::StatusOr<std::shared_ptr<SpawnerNode>> Create();
  virtual ~SpawnerNode();

  // Get the generated camera node names
  std::vector<std::string> GetCameraNodeNames() const;

 private:
  mutable absl::Mutex cameras_mutex_;

  rclcpp::TimerBase::SharedPtr timer_;

  void RefreshCameraList(const std::string& file_camera_path = "");
  void ShutdownCameraNodes();

  std::vector<std::shared_ptr<flowstate_zivid::AdapterNode>> spawned_nodes_;
  std::vector<std::thread> camera_threads_;
  std::vector<std::shared_ptr<Zivid::Camera>> zivid_cameras_
      ABSL_GUARDED_BY(cameras_mutex_);

  std::shared_ptr<Zivid::Application> zivid_app_;
  std::vector<snapshot_interfaces::msg::DiscoveredCamera>
      discovered_camera_msgs_ ABSL_GUARDED_BY(cameras_mutex_);
  rclcpp::Service<snapshot_interfaces::srv::Discover>::SharedPtr
      discover_service_;
};
}  // namespace flowstate_zivid

#endif  // FLOWSTATE_ZIVID_FLOWSTATE_ZIVID_SPAWNER_NODE_H_
