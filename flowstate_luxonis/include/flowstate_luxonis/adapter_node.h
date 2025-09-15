#ifndef FLOWSTATE_LUXONIS_FLOWSTATE_LUXONIS_ADAPTER_NODE_H_
#define FLOWSTATE_LUXONIS_FLOWSTATE_LUXONIS_ADAPTER_NODE_H_

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
// #include "luxonis_camera/ob_camera_node_driver.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "snapshot_interfaces/msg/image_snapshot.hpp"
#include "snapshot_interfaces/srv/describe.hpp"
#include "snapshot_interfaces/srv/snapshot.hpp"

namespace flowstate_luxonis {

class AdapterNode : public rclcpp::Node {
 public:
  AdapterNode(const std::string& serial, const std::string& ip_address);

  bool HasExitedThread() const { return exited_thread_; }
  bool HasSerial(const std::string& serial) const {
    return serial_ == serial;
  }
  std::string GetSerial() const { return serial_; }

 private:
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

  std::string ColorImageTopic() const;
  std::string IrImageTopic() const;
  std::string DepthImageTopic() const;

  absl::Status Main();

  std::string serial_;
  std::string ip_address_;
  std::thread thread_;
  bool exited_thread_ = false;
#if 0
  std::unique_ptr<luxonis_camera::OBCameraNodeDriver> luxonis_node_;
#endif
  rclcpp::Service<snapshot_interfaces::srv::Describe>::SharedPtr
      describe_service_;
  rclcpp::Service<snapshot_interfaces::srv::Snapshot>::SharedPtr
      snapshot_service_;
#if 0
  mutable absl::Mutex camera_info_mutex_;
  std::unique_ptr<sensor_msgs::msg::CameraInfo> color_camera_info_;
  std::unique_ptr<sensor_msgs::msg::CameraInfo> ir_camera_info_;
  std::unique_ptr<sensor_msgs::msg::CameraInfo> depth_camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr ir_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;

  mutable absl::Mutex image_mutex_;
  std::unique_ptr<sensor_msgs::msg::Image> color_image_;
  std::unique_ptr<sensor_msgs::msg::Image> ir_image_;
  std::unique_ptr<sensor_msgs::msg::Image> depth_image_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ir_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
#endif
  mutable absl::Mutex timeout_mutex_;
  rclcpp::Time t_last_color_image_;
};

}  // namespace flowstate_luxonis

#endif  // FLOWSTATE_LUXONIS_FLOWSTATE_LUXONIS_ADAPTER_NODE_H_
