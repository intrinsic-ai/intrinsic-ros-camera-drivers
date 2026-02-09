#include "flowstate_luxonis/adapter_node.h"

#include <memory>

#include "absl/strings/str_format.h"
#include "flowstate_common/image_utils.h"
#include "rclcpp/rclcpp.hpp"

namespace flowstate_luxonis {

AdapterNode::AdapterNode(const std::string& serial,
                         const std::string& ip_address)
    : flowstate_common::CameraAdapterNode(serial, ip_address, "luxonis") {
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

  // Subscribe to camera info
  color_info_sub_ = SubscribeToCameraInfo(
      absl::StrFormat("luxonis/camera_%s/driver/rgb/camera_info", serial_),
      [this](sensor_msgs::msg::CameraInfo::UniquePtr msg) {
        absl::MutexLock lock(&this->camera_info_mutex_);
        this->color_camera_info_ = std::move(msg);
      });

  // Subscribe to color image
  color_image_sub_ = SubscribeToImage(
      ColorImageTopic(),
      [this](sensor_msgs::msg::Image::UniquePtr msg) {
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
  executor.add_node(luxonis_node_);
  executor.spin();
  
  RCLCPP_INFO(this->get_logger(), "Destroying luxonis_camera_node...");
  luxonis_node_.reset();
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  RCLCPP_INFO(this->get_logger(), "Done destroying luxonis_camera_node");
  
  return absl::OkStatus();
}

bool AdapterNode::BuildDescribeResponse(
    snapshot_interfaces::srv::Describe::Response& response) {
  // Lock and check camera info
  absl::MutexLock lock(&camera_info_mutex_);
  if (!color_camera_info_) {
    response.error_message = "CameraInfo not yet received from camera";
    return false;
  }

  AppendSensorDescription(response, *color_camera_info_, "color", ColorImageTopic());
  
  return true;
}

bool AdapterNode::BuildSnapshotResponse(
    snapshot_interfaces::srv::Snapshot::Response& response) {
  
  snapshot_interfaces::msg::ImageSnapshot color_snapshot;
  color_snapshot.topic_name = ColorImageTopic();

  // Lock and copy the most recent CameraInfo messages
  {
    absl::MutexLock lock(&camera_info_mutex_);
    if (!color_camera_info_) {
      response.error_message = "CameraInfo not yet received from camera";
      return false;
    }
    color_snapshot.camera_info = *color_camera_info_;
  }

  // Lock and copy the most recent Image messages
  absl::MutexLock lock(&image_mutex_);
  if (!color_image_) return false;

  // The rgb.i_color_order parameter didn't seem to change the data, so we
  // need to convert BGR->RGB here, as the Flowstate ROS Image Source can
  // only handle rgb8, not bgr8.
  color_snapshot.image.header = color_image_->header;
  color_snapshot.image.height = color_image_->height;
  color_snapshot.image.width = color_image_->width;
  color_snapshot.image.encoding = "rgb8";
  color_snapshot.image.is_bigendian = false;
  color_snapshot.image.step = color_image_->step;
  color_snapshot.image.data.resize(color_snapshot.image.width *
                                   color_snapshot.image.step);

  cv::Mat bgr_image(color_image_->height, color_image_->width, CV_8UC3,
                    const_cast<uint8_t*>(color_image_->data.data()), 
                    color_image_->step);
  cv::Mat rgb_image(color_snapshot.image.height, color_snapshot.image.width, CV_8UC3,
                    color_snapshot.image.data.data(), 
                    color_snapshot.image.step);
  cv::cvtColor(bgr_image, rgb_image, cv::COLOR_BGR2RGB);

  response.images.push_back(std::move(color_snapshot));

  return true;
}

}  // namespace flowstate_luxonis
