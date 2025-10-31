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

namespace zivid_camera {
class ZividCamera;
}

namespace Zivid {
class Application;
class Camera;
class CameraIntrinsics;
struct ColorRGBA;
struct ColorSRGB;
class Frame;
class Frame2D;
template <typename T>
class Image;
class PointCloud;
class Settings2D;
class Settings;
}  // namespace Zivid

namespace flowstate_zivid {
class SpawnerNode : public rclcpp::Node {
 public:
  SpawnerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  static absl::StatusOr<std::shared_ptr<SpawnerNode>> Create();
  virtual ~SpawnerNode();

  // Get the generated camera node names
  std::vector<std::string> getCameraNodeNames() const;

 private:
  mutable absl::Mutex cameras_mutex_;

  rclcpp::TimerBase::SharedPtr timer_;

  void refreshCameraList(const std::string& file_camera_path = "");
  void shutdownCameraNodes();

  std::vector<std::shared_ptr<flowstate_zivid::AdapterNode>> spawned_nodes_;
  std::vector<std::thread> camera_threads_;
  std::vector<std::shared_ptr<Zivid::Camera>> cameras_discovered_
      ABSL_GUARDED_BY(cameras_mutex_);

  std::shared_ptr<Zivid::Application> zivid_app_;
  std::vector<snapshot_interfaces::msg::DiscoveredCamera> cameras_
      ABSL_GUARDED_BY(cameras_mutex_);
  rclcpp::Service<snapshot_interfaces::srv::Discover>::SharedPtr
      discover_service_;
};
}  // namespace flowstate_zivid

#endif  // FLOWSTATE_ZIVID_FLOWSTATE_ZIVID_SPAWNER_NODE_H_
