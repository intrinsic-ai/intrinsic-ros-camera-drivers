#ifndef FLOWSTATE_ZIVID_FLOWSTATE_ZIVID_ADAPTER_NODE_H_
#define FLOWSTATE_ZIVID_FLOWSTATE_ZIVID_ADAPTER_NODE_H_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "snapshot_interfaces/msg/image_snapshot.hpp"
#include "snapshot_interfaces/srv/describe.hpp"
#include "snapshot_interfaces/srv/snapshot.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace Zivid {
class Application;
class Camera;
class Settings;
}  // namespace Zivid
namespace zivid_camera {
class ZividCamera;
}  // namespace zivid_camera
namespace flowstate_zivid {

struct ZividCaptureParameters {
  double exposure_time;
  double gain;
  double gamma;
  double projector_brightness;
  double aperture;
  bool outlier_removal_enabled;
  double outlier_removal_threshold;

  static ZividCaptureParameters boot_defaults() {
    return {
        8333,  // exposure_time (8333us)
        1.0,   // gain
        1.0,   // gamma
        1.0,   // projector_brightness
        5.66,  // aperture
        true,  // outlier_removal_enabled
        5.0    // outlier_removal_threshold
    };
  };
};

class AdapterNode : public rclcpp::Node {
 public:
  AdapterNode(const std::string& serial, const rclcpp::NodeOptions& options,
              std::shared_ptr<Zivid::Camera> camera,
              std::shared_ptr<Zivid::Application> zivid_app);

  bool HasExitedThread() const { return exited_thread_; }

  bool HasSerial(const std::string& serial) const { return serial_ == serial; }

  std::string GetSerial() const { return serial_; }

 private:
  rcl_interfaces::msg::SetParametersResult setParametersCallback(
      const std::vector<rclcpp::Parameter>& parameters);
  void updateSettingsYamlCallback();
  std::string generateZividSettings() const;

  void DescribeCallback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<snapshot_interfaces::srv::Describe::Request>
          request,
      const std::shared_ptr<snapshot_interfaces::srv::Describe::Response>
          response);

  void SnapshotCallback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<snapshot_interfaces::srv::Snapshot::Request>
          request,
      const std::shared_ptr<snapshot_interfaces::srv::Snapshot::Response>
          response);

  absl::Status Main();

  std::string ColorImageTopic() const;

  std::string DepthImageTopic() const;

  std::string CameraInfoTopic() const;

  std::string NormalTopic() const;

  std::string serial_;

  ZividCaptureParameters capture_params_;
  std::thread thread_;
  bool exited_thread_ = false;
  std::unique_ptr<zivid_camera::ZividCamera> zivid_node_;
  std::thread zivid_node_thread_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor>
      zivid_node_executor_;

  // ROS Services
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Service<snapshot_interfaces::srv::Describe>::SharedPtr
      describe_service_;
  rclcpp::Service<snapshot_interfaces::srv::Snapshot>::SharedPtr
      snapshot_service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr capture_client_;

  // Mutex-protected CameraInfo storage and subscriptions
  std::mutex camera_info_mutex_;
  sensor_msgs::msg::CameraInfo::UniquePtr camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
      camera_info_sub_;

  // Mutex-protected Image storage and subscriptions
  std::mutex data_mutex_;
  sensor_msgs::msg::Image::UniquePtr color_image_;
  sensor_msgs::msg::Image::UniquePtr depth_image_;
  sensor_msgs::msg::PointCloud2::UniquePtr normal_pc_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr normal_sub_;

  // Snapshot synchronization
  std::mutex snapshot_mutex_;
  std::condition_variable snapshot_cv_;
  bool new_color_image_received_{false};
  bool new_depth_image_received_{false};
  bool new_normal_pc_received_{false};

  // Timeout monitoring
  std::mutex timeout_mutex_;
  rclcpp::Time t_last_color_image_;

  rclcpp::TimerBase::SharedPtr update_settings_yaml_timer_;
  std::atomic<bool> individual_settings_dirty_{false};
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr
      set_parameters_callback_handle_;
  mutable std::mutex capture_params_mutex_;
  std::shared_ptr<rclcpp::AsyncParametersClient> zivid_camera_param_client_;
};

}  // namespace flowstate_zivid

#endif  // FLOWSTATE_ZIVID_FLOWSTATE_ZIVID_ADAPTER_NODE_H_
