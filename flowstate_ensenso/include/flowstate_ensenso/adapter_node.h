#ifndef FLOWSTATE_ENSENSO_FLOWSTATE_ENSENSO_ADAPTER_NODE_H_
#define FLOWSTATE_ENSENSO_FLOWSTATE_ENSENSO_ADAPTER_NODE_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flowstate_common/base_adapter_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "snapshot_interfaces/srv/describe.hpp"
#include "snapshot_interfaces/srv/snapshot.hpp"

namespace flowstate_ensenso {

class AdapterNode : public flowstate_common::BaseAdapterNode {
 public:
  AdapterNode(const std::string& serial,
              const std::vector<std::string>& locators);

 private:
  absl::Status Main() override ABSL_LOCKS_EXCLUDED(timeout_mutex_);
  std::string ColorImageTopic() const override;
  absl::StatusOr<snapshot_interfaces::srv::Describe::Response>
  BuildDescribeResponse() override;
  absl::StatusOr<snapshot_interfaces::srv::Snapshot::Response>
  BuildSnapshotResponse() override;

  std::string DepthImageTopic() const;
  std::string LeftCameraInfoTopic() const;
  std::string DepthCameraInfoTopic() const;

  std::unique_ptr<sensor_msgs::msg::CameraInfo> left_camera_info_
      ABSL_GUARDED_BY(camera_info_mutex_);
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
      left_info_sub_;
  std::unique_ptr<sensor_msgs::msg::Image> left_image_
      ABSL_GUARDED_BY(image_mutex_);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_image_sub_;

  std::unique_ptr<sensor_msgs::msg::CameraInfo> depth_camera_info_
      ABSL_GUARDED_BY(camera_info_mutex_);
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
      depth_info_sub_;
  std::unique_ptr<sensor_msgs::msg::Image> depth_image_
      ABSL_GUARDED_BY(image_mutex_);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
};

}  // namespace flowstate_ensenso

#endif  // FLOWSTATE_ENSENSO_FLOWSTATE_ENSENSO_ADAPTER_NODE_H_
