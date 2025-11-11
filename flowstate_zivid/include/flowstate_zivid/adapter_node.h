#ifndef FLOWSTATE_ZIVID_FLOWSTATE_ZIVID_ADAPTER_NODE_H_
#define FLOWSTATE_ZIVID_FLOWSTATE_ZIVID_ADAPTER_NODE_H_

#include <condition_variable>
#include <memory>
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
#include "zivid_camera/zivid_camera.hpp"

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

/**
 * @class AdapterNode
 * @brief A ROS 2 node that acts as an adapter for the zivid_camera::ZividCamera
 * node from the official Zivid ROS driver
 * (https://github.com/intrinsic-dev/zivid-ros/blob/flowstate/zivid_camera/include/zivid_camera/zivid_camera.hpp).
 *
 * The AdapterNode provides a simplified, high-level interface for interacting
 * with a `zivid_camera::ZividCamera` node, which handles the direct hardware
 * communication.
 *
 * -----
 * Key Responsibilities:
 * 1.  Encapsulation:
 *     It creates and manages a zivid_camera::ZividCamera
 *     node instance in an internal thread, hiding its detailed implementation.
 *
 * 2.  Parameter Abstraction:
 *     It exposes capture parameters for Flowstate to set parameters during
 *     runtime. When these are modified, it translates them into the full YAML
 *     configuration required by the underlying `zivid_camera` node.
 *
 * 3.  Flowstate Service Interface:
 *     It offers support for the `snapshot_interfaces` services from Flowstate
 *     sdk-ros:
 *     - "~/snapshot": Triggers a 3D capture, collects all resulting data
 *       (color,depth images, normals), and returns them in a single,
 *       synchronized response.
 *     - "~/describe": Provides a structured description of the camera's
 *       available sensors and their properties.
 */
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
  /**
   * @brief (Re)starts the capture timer with a new frame rate.
   */
  void onCaptureTimer(double fps);
  /**
   * @brief Generates a Zivid settings string in YAML format.
   * @return A string containing the Zivid settings in YAML format.
   */
  std::string GenerateZividSettings() const;

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

  absl::CondVar snapshot_cv_;
  ZividCaptureParameters capture_params_;
  bool exited_thread_ = false;

  // ROS Services
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Service<snapshot_interfaces::srv::Describe>::SharedPtr
      describe_service_;
  rclcpp::Service<snapshot_interfaces::srv::Snapshot>::SharedPtr
      snapshot_service_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr capture_client_;
  rclcpp::TimerBase::SharedPtr capture_timer_;

  std::unique_ptr<zivid_camera::ZividCamera> zivid_node_;

  mutable absl::Mutex camera_info_mutex_;
  sensor_msgs::msg::CameraInfo::UniquePtr camera_info_
      ABSL_GUARDED_BY(camera_info_mutex_);
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
      camera_info_sub_;

  mutable absl::Mutex data_mutex_;
  sensor_msgs::msg::Image::UniquePtr color_image_ ABSL_GUARDED_BY(data_mutex_);
  sensor_msgs::msg::Image::UniquePtr depth_image_ ABSL_GUARDED_BY(data_mutex_);
  sensor_msgs::msg::PointCloud2::UniquePtr normal_pc_
      ABSL_GUARDED_BY(data_mutex_);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr normal_sub_;

  // Timeout monitoring
  mutable absl::Mutex timeout_mutex_;
  rclcpp::Time t_last_color_image_ ABSL_GUARDED_BY(timeout_mutex_);

  rclcpp::TimerBase::SharedPtr update_settings_yaml_timer_;

  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr
      set_parameters_callback_handle_;
  mutable absl::Mutex capture_params_mutex_;
  std::shared_ptr<rclcpp::AsyncParametersClient> zivid_camera_param_client_;

  std::thread thread_;
};

}  // namespace flowstate_zivid

#endif  // FLOWSTATE_ZIVID_FLOWSTATE_ZIVID_ADAPTER_NODE_H_
