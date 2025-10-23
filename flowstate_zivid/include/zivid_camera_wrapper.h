#include <rclcpp/rclcpp.hpp>
#include "snapshot_interfaces/srv/discover.hpp"
#include "snapshot_interfaces/msg/discovered_camera.hpp"
#include <absl/status/status.h>
#include <absl/status/statusor.h>
#include <absl/strings/str_cat.h>
#include <absl/synchronization/mutex.h>
#include <thread>
#include "zivid_camera_node.h"

// Forward declarations
namespace zivid_camera {
  class ZividCamera;
}

namespace Zivid
{
class Application;
class Camera;
class CameraIntrinsics;
struct ColorRGBA;
struct ColorSRGB;
using ColorRGBA_SRGB = ColorSRGB;
class Frame;
class Frame2D;
template <typename T>
class Image;
class PointCloud;
class Settings2D;
class Settings;
}  // namespace Zivid/zivid_camera_node.h
namespace zivid_camera_wrapper
{

class ZividCameraWrapper : public rclcpp::Node
{
public:
  ZividCameraWrapper(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  static absl::StatusOr<std::shared_ptr<ZividCameraWrapper>> Create();
  virtual ~ZividCameraWrapper();
  
  // Get the generated camera node names
  std::vector<std::string> getCameraNodeNames() const;

private:
  struct CameraInfo {
    std::string serial_number;
    std::string model_name;
    bool is_available;
    bool is_connected;
  };

  mutable absl::Mutex cameras_mutex_;

  rclcpp::TimerBase::SharedPtr timer_;

  void refreshCameraList(const std::string & file_camera_path = "");
  void shutdownCameraNodes();
  
  std::vector<std::shared_ptr<zivid_camera_node::ZividCamNode>> camera_nodes_;
  std::vector<std::thread> camera_threads_;

  std::unique_ptr<Zivid::Application> zivid_app_;
  std::vector<snapshot_interfaces::msg::DiscoveredCamera> cameras_
    ABSL_GUARDED_BY(cameras_mutex_);
  rclcpp::Service<snapshot_interfaces::srv::Discover>::SharedPtr discover_service_;
};
}  // namespace zivid_camera_wrapper