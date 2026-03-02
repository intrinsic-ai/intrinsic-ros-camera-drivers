#include "flowstate_luxonis/adapter_node.h"

#include <memory>

#include "absl/strings/str_format.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "snapshot_interfaces/msg/image_snapshot.hpp"

namespace flowstate_luxonis {

AdapterNode::AdapterNode(const std::string& serial,
                         const std::vector<std::string>& locators)
    : flowstate_common::BaseAdapterNode(serial, locators, "luxonis") {
  // ip address should be the first locator if it exists
  std::string ip_address = locators.empty() ? "" : locators[0];

  const std::string luxonis_node_name = std::string("luxonis_camera_node");
  const std::string luxonis_ns = std::string("/luxonis/camera_") + serial;
  rclcpp::NodeOptions luxonis_node_options =
      rclcpp::NodeOptions()
          .arguments({"--ros-args", "-r", "__ns:=" + luxonis_ns})
          .append_parameter_override(
              rclcpp::Parameter("driver.i_ip", ip_address))
          .append_parameter_override(
              rclcpp::Parameter("driver.r_laser_dot_intensity", 0.0))
          .append_parameter_override(
              rclcpp::Parameter("driver.r_floodlight_intensity", 0.0))
          .append_parameter_override(
              rclcpp::Parameter("pipeline_gen.i_pipeline_type", "RGB"))
          .append_parameter_override(
              rclcpp::Parameter("pipeline_gen.i_nn_type", "none"))
          .append_parameter_override(
              rclcpp::Parameter("pipeline_gen.i_enable_imu", false))
          .append_parameter_override(rclcpp::Parameter("rgb.i_fps", 10.0))
          .append_parameter_override(rclcpp::Parameter("rgb.i_width", 1280))
          .append_parameter_override(rclcpp::Parameter("rgb.i_height", 800))
          .append_parameter_override(
              rclcpp::Parameter("rgb.i_low_bandwidth", false));

  luxonis_node_ =
      std::make_shared<depthai_ros_driver::Driver>(luxonis_node_options);

  color_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      absl::StrFormat("luxonis/camera_%s/driver/rgb/camera_info", serial_), 2,
      [this](sensor_msgs::msg::CameraInfo::UniquePtr msg) {
        absl::MutexLock lock(&this->camera_info_mutex_);
        this->color_camera_info_ = std::move(msg);
      });

  color_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      ColorImageTopic(), 2, [this](sensor_msgs::msg::Image::UniquePtr msg) {
        {
          absl::MutexLock timeout_lock(&this->timeout_mutex_);
          this->t_last_color_image_ = this->get_clock()->now();
        }
        absl::MutexLock lock(&this->image_mutex_);
        this->color_image_ = std::move(msg);
      });

  // Create Flowstate services
  CreateFlowstateServices();

  // Start background executor thread
  StartExecutorThread();
}

std::string AdapterNode::ColorImageTopic() const {
  return absl::StrFormat("/luxonis/camera_%s/driver/rgb/image_raw", serial_);
}

absl::Status AdapterNode::Main() {
  RCLCPP_INFO(get_logger(), "AdapterNode::Main()");
  t_last_color_image_ = get_clock()->now();
  rclcpp::executors::SingleThreadedExecutor executor;

  liveness_timer_ =
      create_wall_timer(std::chrono::seconds(1), [this, &executor]() {
        absl::MutexLock timeout_lock(&timeout_mutex_);
        if ((get_clock()->now() - t_last_color_image_).seconds() > 30.0) {
          RCLCPP_ERROR(get_logger(), "No new image arrived for 30 seconds");
          executor.cancel();
        }
      });

  executor.add_node(this->get_node_base_interface());
  executor.add_node(luxonis_node_);  //->get_node_base_interface());
  executor.spin();
  return absl::OkStatus();
}

absl::StatusOr<snapshot_interfaces::srv::Describe::Response>
AdapterNode::BuildDescribeResponse(
    snapshot_interfaces::srv::Describe::Response& response) {
  snapshot_interfaces::srv::Describe::Response response;
  auto color_camera_info_ = GetColorCameraInfo();
  if (!color_camera_info_) {
    return absl::UnavailableError("CameraInfo not yet received from camera");
  }

  response.sensors.push_back(
      SensorInformation(*color_camera_info_, "color", ColorImageTopic()));
  return response;
}

absl::StatusOr<snapshot_interfaces::srv::Snapshot::Response>
AdapterNode::BuildSnapshotResponse(
    snapshot_interfaces::srv::Snapshot::Response& response) {
  snapshot_interfaces::msg::ImageSnapshot color_snapshot;
  // Note that we'll need something smarter in order to be able to implement
  // WAIT_FOR_NEXT; a single-threaded executor will never be able to block
  // here while waiting for the image message callbacks to be invoked.
  color_snapshot.topic_name = ColorImageTopic();

  auto info_copy = GetColorCameraInfo();
  if (!info_copy) {
    return absl::UnavailableError("CameraInfo not yet received from camera");
  }
  color_snapshot.camera_info = *info_copy;

  auto img_copy = GetColorImage();
  if (!img_copy) {
    return absl::UnavailableError("images not yet received from camera");
  }
  // The rgb.i_color_order parameter didn't seem to change the data, so we
  // need to convert BGR->RGB here, as the Flowstate ROS Image Source can
  // only handle rgb8, not bgr8.
  color_snapshot.image.header = img_copy->header;
  color_snapshot.image.height = img_copy->height;
  color_snapshot.image.width = img_copy->width;
  color_snapshot.image.encoding = "rgb8";
  color_snapshot.image.is_bigendian = false;
  color_snapshot.image.step = img_copy->step;
  color_snapshot.image.data.resize(color_snapshot.image.width *
                                   color_snapshot.image.step);

  const cv::Mat bgr_image(img_copy->height, img_copy->width, CV_8UC3,
                          img_copy->data.data(), img_copy->step);
  cv::Mat rgb_image(color_snapshot.image.height, color_snapshot.image.width,
                    CV_8UC3, color_snapshot.image.data.data(), img_copy->step);

  cv::cvtColor(bgr_image, rgb_image, cv::COLOR_BGR2RGB);

  response.images.push_back(std::move(color_snapshot));
  return response;
}

}  // namespace flowstate_luxonis
