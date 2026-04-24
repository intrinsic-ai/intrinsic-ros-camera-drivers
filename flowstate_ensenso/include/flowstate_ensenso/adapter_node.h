#ifndef FLOWSTATE_ENSENSO_FLOWSTATE_ENSENSO_ADAPTER_NODE_H_
#define FLOWSTATE_ENSENSO_FLOWSTATE_ENSENSO_ADAPTER_NODE_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "flowstate_common/base_adapter_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "snapshot_interfaces/srv/describe.hpp"
#include "snapshot_interfaces/srv/snapshot.hpp"
#include "ensenso_camera_msgs/action/request_data.hpp"

namespace flowstate_ensenso {

class AdapterNode : public flowstate_common::BaseAdapterNode {
 public:
  AdapterNode(const std::string& serial,
              const std::vector<std::string>& locators);

 private:
  absl::Status Main() override;
  std::string ColorImageTopic() const override;
  absl::StatusOr<snapshot_interfaces::srv::Describe::Response>
  BuildDescribeResponse() override ABSL_LOCKS_EXCLUDED(data_mutex_);
  absl::StatusOr<snapshot_interfaces::srv::Snapshot::Response>
  BuildSnapshotResponse() override ABSL_LOCKS_EXCLUDED(data_mutex_);

  std::string DepthImageTopic() const;
  std::string LeftCameraInfoTopic() const;
  std::string DepthCameraInfoTopic() const;

  struct CaptureData {
    sensor_msgs::msg::Image::UniquePtr left_image;
    sensor_msgs::msg::Image::UniquePtr depth_image;
    std::unique_ptr<sensor_msgs::msg::CameraInfo> left_camera_info;
    std::unique_ptr<sensor_msgs::msg::CameraInfo> depth_camera_info;
  };

  /**
   * @brief Triggers a capture via the Ensenso Action Server and waits for data.
   * @param only_info_needed If true, only waits for CameraInfo (used by Describe).
   * @return CaptureData on success, or error status on failure.
   */
  absl::StatusOr<CaptureData> Capture(bool only_info_needed = false) ABSL_LOCKS_EXCLUDED(data_mutex_);

  std::unique_ptr<rclcpp::Node> ensenso_node_;

  // Ensenso uses an Action instead of a Service to trigger data
  rclcpp_action::Client<ensenso_camera_msgs::action::RequestData>::SharedPtr request_data_client_;

  mutable absl::Mutex data_mutex_;
  CaptureData data_ ABSL_GUARDED_BY(data_mutex_);
};

}  // namespace flowstate_ensenso

#endif  // FLOWSTATE_ENSENSO_FLOWSTATE_ENSENSO_ADAPTER_NODE_H_
