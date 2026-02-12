#ifndef FLOWSTATE_ZIVID_FLOWSTATE_ZIVID_SPAWNER_NODE_H_
#define FLOWSTATE_ZIVID_FLOWSTATE_ZIVID_SPAWNER_NODE_H_

#include <memory>
#include <vector>
#include <absl/status/statusor.h>

#include "flowstate_common/camera_spawner_node.h"
#include "flowstate_zivid/adapter_node.h"
#include <Zivid/Application.h>

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
 *     cameras.
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
class SpawnerNode : public flowstate_common::CameraSpawnerNode {
 public:
  SpawnerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  static absl::StatusOr<std::shared_ptr<SpawnerNode>> Create();

  // Get the generated camera node names
  std::vector<std::string> GetCameraNodeNames() const;

 protected:
  void UpdateCameras() override;

 private:
  void ShutdownCameraNodes();
  std::shared_ptr<Zivid::Application> zivid_app_;
};
}  // namespace flowstate_zivid

#endif  // FLOWSTATE_ZIVID_FLOWSTATE_ZIVID_SPAWNER_NODE_H_
