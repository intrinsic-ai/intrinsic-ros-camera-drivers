#include "flowstate_luxonis/adapter_node.h"

#include <memory>

#include "opencv2/core.hpp"
// #include "luxonis_camera/ob_camera_node_driver.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "snapshot_interfaces/msg/image_snapshot.hpp"

namespace flowstate_luxonis {

using snapshot_interfaces::srv::Describe;
using snapshot_interfaces::srv::Snapshot;

AdapterNode::AdapterNode(const std::string& serial,
                         const std::string& ip_address)
    : Node(std::string("luxonis_") + serial),
      serial_(serial),
      ip_address_(ip_address) {
  const std::string luxonis_node_name = std::string("luxonis_camera_node");
  const std::string luxonis_ns = std::string("/luxonis/camera_") + serial;
  rclcpp::NodeOptions luxonis_node_options =
      rclcpp::NodeOptions()
          .arguments({"--ros-args", "-r", "__ns:=" + luxonis_ns})
          .append_parameter_override(
              rclcpp::Parameter("camera.i_ip", ip_address))
          .append_parameter_override(
              rclcpp::Parameter("camera.i_laser_dot_brightness", 0))
          .append_parameter_override(
              rclcpp::Parameter("camera.i_pipeline_type", "RGB"))
          .append_parameter_override(
              rclcpp::Parameter("camera.i_nn_type", "none"))
          .append_parameter_override(
              rclcpp::Parameter("pipeline_gen.i_enable_imu", false))
          .append_parameter_override(rclcpp::Parameter("rgb.i_fps", 10.0))
          .append_parameter_override(
              rclcpp::Parameter("rgb.i_low_bandwidth", false));
  luxonis_node_ =
      std::make_shared<depthai_ros_driver::Camera>(luxonis_node_options);

  color_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      absl::StrFormat("luxonis/camera_%s/camera/rgb/camera_info", serial_), 2,
      [this](sensor_msgs::msg::CameraInfo::UniquePtr msg) {
        absl::MutexLock lock(&this->camera_info_mutex_);
        this->color_camera_info_ = std::move(msg);
      });
  color_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      ColorImageTopic(), 2, [this](sensor_msgs::msg::Image::UniquePtr msg) {
        // RCLCPP_INFO(this->get_logger(), "Received color image");
        {
          absl::MutexLock timeout_lock(&this->timeout_mutex_);
          this->t_last_color_image_ = this->get_clock()->now();
        }
        absl::MutexLock lock(&this->image_mutex_);
        this->color_image_ = std::move(msg);
      });

  describe_service_ = create_service<Describe>(
      "~/describe",
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
             const std::shared_ptr<Describe::Request> request,
             const std::shared_ptr<Describe::Response> response) {
        this->DescribeCallback(request_header, request, response);
      });
  snapshot_service_ = create_service<Snapshot>(
      "~/snapshot",
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
             const std::shared_ptr<Snapshot::Request> request,
             const std::shared_ptr<Snapshot::Response> response) {
        this->SnapshotCallback(request_header, request, response);
      });

  thread_ = std::thread([this]() {
    const absl::Status status = this->Main();
    if (!status.ok()) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "node thread error: " << status);
    }
    RCLCPP_INFO(this->get_logger(), "Destroying luxonis_camera_node...");
    luxonis_node_.reset();
    rclcpp::sleep_for(std::chrono::milliseconds(500));  // maybe this helps?
    RCLCPP_INFO(this->get_logger(), "Done destroying luxonis_camera_node");
    exited_thread_ = true;
  });
}

std::string AdapterNode::ColorImageTopic() const {
  return absl::StrFormat("/luxonis/camera_%s/camera/rgb/image_raw", serial_);
}

absl::Status AdapterNode::Main() {
  RCLCPP_INFO(get_logger(), "AdapterNode::Main()");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(this->get_node_base_interface());
  executor.add_node(luxonis_node_); //->get_node_base_interface());
  // TODO: add some other tests for camera health, to exit this loop if it's bad
  t_last_color_image_ = get_clock()->now();
  while (rclcpp::ok()) {
    executor.spin_some();  // todo: something smarter
    // maybe do something
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    {
      absl::MutexLock timeout_lock(&timeout_mutex_);
      if ((get_clock()->now() - t_last_color_image_).seconds() > 30.0) {
        RCLCPP_ERROR(get_logger(), "No new image arrived for 30 seconds");
        break;
      }
    }
  }
  return absl::OkStatus();
}

void AdapterNode::DescribeCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<snapshot_interfaces::srv::Describe::Request>,
    const std::shared_ptr<snapshot_interfaces::srv::Describe::Response>
        response) {
  RCLCPP_INFO(get_logger(), "AdapterNode::DescribeCallback()");

  absl::MutexLock lock(&camera_info_mutex_);
  if (!color_camera_info_) {
    response->error_message = "CameraInfo not yet received from camera";
    response->success = false;
    RCLCPP_ERROR(get_logger(), response->error_message.c_str());
    return;
  }

  snapshot_interfaces::msg::SensorInfo color_info;
  color_info.sensor_name = "color";
  color_info.topic_name = ColorImageTopic();
  color_info.sensor_type = snapshot_interfaces::msg::SensorInfo::IMAGE;
  color_info.camera_t_sensor.transform.rotation.w = 1.0;  // todo: get static transform
  color_info.info.push_back(*color_camera_info_);
  response->sensors.push_back(color_info);

  response->success = true;
}

void AdapterNode::SnapshotCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<snapshot_interfaces::srv::Snapshot::Request> /*request*/,
    const std::shared_ptr<snapshot_interfaces::srv::Snapshot::Response>
        response) {
  // Note that we'll need something smarter in order to be able to implement
  // WAIT_FOR_NEXT; a single-threaded executor will never be able to block
  // here while waiting for the image message callbacks to be invoked.
  snapshot_interfaces::msg::ImageSnapshot color_snapshot;

  color_snapshot.topic_name = ColorImageTopic();

  // Lock and copy the most recent CameraInfo messages
  {
    absl::MutexLock lock(&camera_info_mutex_);
    if (!color_camera_info_) {
      response->error_message = "CameraInfo not yet received";
      response->success = false;
      RCLCPP_ERROR(get_logger(), response->error_message.c_str());
      return;
    }
    color_snapshot.camera_info = *color_camera_info_;
  }

  // Lock and copy the most recent Image messages
  absl::MutexLock lock(&image_mutex_);
  if (!color_image_) {
    response->error_message = "images not yet received from camera";
    response->success = false;
    RCLCPP_ERROR(get_logger(), response->error_message.c_str());
    return;
  }

  color_snapshot.image = *color_image_;
  response->images.push_back(std::move(color_snapshot));

  response->success = true;
}

}  // namespace flowstate_luxonis
