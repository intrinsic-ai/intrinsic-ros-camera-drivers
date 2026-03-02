#ifndef FLOWSTATE_ZIVID_FLOWSTATE_ZIVID_ADAPTER_NODE_H_
#define FLOWSTATE_ZIVID_FLOWSTATE_ZIVID_ADAPTER_NODE_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "flowstate_common/base_adapter_node.h"
#include "image_transport/image_transport.hpp"
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
 *     node instance.
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
class AdapterNode : public flowstate_common::BaseAdapterNode {
 public:
  AdapterNode(const std::string& serial, const rclcpp::NodeOptions& options,
              std::shared_ptr<Zivid::Application> zivid_app);

 private:
  absl::Status Main() override;
  std::string ColorImageTopic() const override;
  absl::StatusOr<snapshot_interfaces::srv::Describe::Response>
  BuildDescribeResponse(
      snapshot_interfaces::srv::Describe::Response& response) override;
  absl::StatusOr<snapshot_interfaces::srv::Snapshot::Response>
  BuildSnapshotResponse(
      snapshot_interfaces::srv::Snapshot::Response& response) override;

  // --- Zivid Specific Implementations ---
  std::string DepthImageTopic() const;
  std::string NormalTopic() const;

  void InitializeParameters();
  rcl_interfaces::msg::SetParametersResult SetParametersCallback(
      const std::vector<rclcpp::Parameter>& parameters);
  /**
   * @brief Generates a Zivid settings string in YAML format.
   * @return A string containing the Zivid settings in YAML format.
   */
  std::string GenerateZividSettings() const;

  struct CaptureParameters {
    double exposure_time;
    double gain;
    double gamma;
    double projector_brightness;
    double aperture;
    bool outlier_removal_enabled;
    double outlier_removal_threshold;

    static CaptureParameters boot_defaults() {
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

  struct CaptureData {
    sensor_msgs::msg::Image::ConstSharedPtr color_image;
    sensor_msgs::msg::Image::UniquePtr depth_image;
    sensor_msgs::msg::PointCloud2::UniquePtr normal_pc;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info;

    bool AllAvailable() const {
      return color_image != nullptr && depth_image != nullptr &&
             normal_pc != nullptr && camera_info != nullptr;
    }
  };

  /**
   * @brief Triggers a capture and returns the captured data.
   * @return CaptureData on success, or error status on failure.
   */
  absl::StatusOr<CaptureData> Capture();

  std::unique_ptr<zivid_camera::ZividCamera> zivid_node_;

  // Internal client to trigger the Zivid driver
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr capture_client_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;

  mutable absl::Mutex data_mutex_;
  CaptureData data_ ABSL_GUARDED_BY(data_mutex_);

  image_transport::CameraSubscriber color_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr normal_sub_;

  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr
      set_parameters_callback_handle_;
  mutable absl::Mutex capture_params_mutex_;
  CaptureParameters capture_params_ ABSL_GUARDED_BY(capture_params_mutex_);
  std::shared_ptr<rclcpp::AsyncParametersClient> zivid_camera_param_client_;
};

}  // namespace flowstate_zivid

#endif  // FLOWSTATE_ZIVID_FLOWSTATE_ZIVID_ADAPTER_NODE_H_
