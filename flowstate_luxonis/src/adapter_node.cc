#include "flowstate_luxonis/adapter_node.h"

#include <memory>

#include "absl/strings/str_format.h"
#include "flowstate_common/image_utils.h"
#include "rclcpp/rclcpp.hpp"

namespace flowstate_luxonis {

AdapterNode::AdapterNode(const std::string& serial,
                         const std::string& ip_address)
    : flowstate_common::CameraAdapterNode(serial, ip_address, "luxonis") {
  const std::string luxonis_node_name = "luxonis_camera_node";
  const std::string luxonis_ns = "/luxonis/camera_" + serial;
  
  rclcpp::NodeOptions luxonis_node_options =
      rclcpp::NodeOptions()
          .arguments({"--ros-args", "-r", "__ns:=" + luxonis_ns})
          .append_parameter_override(rclcpp::Parameter("driver.i_ip", ip_address))
          .append_parameter_override(rclcpp::Parameter("driver.r_laser_dot_intensity", 0.0))
          .append_parameter_override(rclcpp::Parameter("driver.r_floodlight_intensity", 0.0))
          .append_parameter_override(rclcpp::Parameter("pipeline_gen.i_pipeline_type", "RGB"))
          .append_parameter_override(rclcpp::Parameter("pipeline_gen.i_nn_type", "none"))
          .append_parameter_override(rclcpp::Parameter("pipeline_gen.i_enable_imu", false))
          .append_parameter_override(rclcpp::Parameter("rgb.i_fps", 10.0))
          .append_parameter_override(rclcpp::Parameter("rgb.i_width", 1280))
          .append_parameter_override(rclcpp::Parameter("rgb.i_height", 800))
          .append_parameter_override(rclcpp::Parameter("rgb.i_low_bandwidth", false));
  
  luxonis_node_ = std::make_shared<depthai_ros_driver::Driver>(luxonis_node_options);

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

}  // namespace flowstate_luxonis
