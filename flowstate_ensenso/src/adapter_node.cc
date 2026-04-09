#include "flowstate_ensenso/adapter_node.h"

#include "absl/strings/str_format.h"
#include "rclcpp/rclcpp.hpp"

namespace flowstate_ensenso {

AdapterNode::AdapterNode(const std::string& serial,
                         const std::vector<std::string>& locators)
    : flowstate_common::BaseAdapterNode(serial, locators, "ensenso") {
  const std::string left_info_topic = LeftCameraInfoTopic();
  const std::string depth_info_topic = DepthCameraInfoTopic();

  left_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      left_info_topic, 2,
      [this](sensor_msgs::msg::CameraInfo::UniquePtr msg) {
        absl::MutexLock lock(&camera_info_mutex_);
        left_camera_info_ = std::move(msg);
      });

  left_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      ColorImageTopic(), 2,
      [this](sensor_msgs::msg::Image::UniquePtr msg) {
        {
          absl::MutexLock timeout_lock(&timeout_mutex_);
          t_last_color_image_ = this->get_clock()->now();
        }
        absl::MutexLock lock(&image_mutex_);
        left_image_ = std::move(msg);
      });

  depth_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      depth_info_topic, 2,
      [this](sensor_msgs::msg::CameraInfo::UniquePtr msg) {
        absl::MutexLock lock(&camera_info_mutex_);
        depth_camera_info_ = std::move(msg);
      });

  depth_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      DepthImageTopic(), 2,
      [this](sensor_msgs::msg::Image::UniquePtr msg) {
        absl::MutexLock lock(&image_mutex_);
        depth_image_ = std::move(msg);
      });

  CreateFlowstateServices();
  StartExecutorThread();
}

std::string AdapterNode::ColorImageTopic() const {
  return absl::StrFormat("/ensenso/camera_%s/rectified/left/image", serial_);
}

std::string AdapterNode::DepthImageTopic() const {
  return absl::StrFormat("/ensenso/camera_%s/depth/image", serial_);
}

std::string AdapterNode::LeftCameraInfoTopic() const {
  return absl::StrFormat("/ensenso/camera_%s/rectified/left/camera_info", serial_);
}

std::string AdapterNode::DepthCameraInfoTopic() const {
  return absl::StrFormat("/ensenso/camera_%s/depth/camera_info", serial_);
}

absl::Status AdapterNode::Main() {
  RCLCPP_INFO(get_logger(), "AdapterNode::Main() for ensenso camera %s",
              serial_.c_str());
  rclcpp::executors::SingleThreadedExecutor executor;

  {
    absl::MutexLock timeout_lock(&timeout_mutex_);
    t_last_color_image_ = get_clock()->now();
  }

  liveness_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this, &executor]() {
        absl::MutexLock timeout_lock(&timeout_mutex_);
        if ((get_clock()->now() - t_last_color_image_).seconds() > 30.0) {
          RCLCPP_ERROR(get_logger(), "No new image arrived for 30 seconds");
          executor.cancel();
        }
      });

  executor.add_node(this->get_node_base_interface());
  executor.spin();
  return absl::OkStatus();
}

absl::StatusOr<snapshot_interfaces::srv::Describe::Response>
AdapterNode::BuildDescribeResponse() {
  snapshot_interfaces::srv::Describe::Response response;
  absl::MutexLock camera_info_lock(&camera_info_mutex_);

  if (!left_camera_info_) {
    return absl::UnavailableError(
        "CameraInfo not yet received from Ensenso left rectified stream");
  }

  response.sensors.push_back(BuildSensorInformation(*left_camera_info_,
                                                    "left_rectified",
                                                    ColorImageTopic()));

  if (depth_camera_info_) {
    response.sensors.push_back(BuildSensorInformation(*depth_camera_info_,
                                                      "depth",
                                                      DepthImageTopic()));
  }

  return response;
}

absl::StatusOr<snapshot_interfaces::srv::Snapshot::Response>
AdapterNode::BuildSnapshotResponse() {
  snapshot_interfaces::srv::Snapshot::Response response;

  absl::MutexLock camera_info_lock(&camera_info_mutex_);
  absl::MutexLock image_lock(&image_mutex_);

  if (!left_camera_info_) {
    return absl::UnavailableError(
        "CameraInfo not yet received from Ensenso left rectified stream");
  }
  if (!left_image_) {
    return absl::UnavailableError(
        "Left rectified image not yet received from Ensenso camera");
  }

  snapshot_interfaces::msg::ImageSnapshot left_snapshot;
  left_snapshot.topic_name = ColorImageTopic();
  left_snapshot.camera_info = *left_camera_info_;
  left_snapshot.image = *left_image_;
  response.images.push_back(std::move(left_snapshot));

  if (depth_camera_info_ && depth_image_) {
    snapshot_interfaces::msg::ImageSnapshot depth_snapshot;
    depth_snapshot.topic_name = DepthImageTopic();
    depth_snapshot.camera_info = *depth_camera_info_;
    depth_snapshot.image = *depth_image_;
    response.images.push_back(std::move(depth_snapshot));
  }

  return response;
}

}  // namespace flowstate_ensenso
