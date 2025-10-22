#include "flowstate_luxonis/adapter_node.h"

#include <memory>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
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
  return absl::StrFormat("/luxonis/camera_%s/driver/rgb/image_raw", serial_);
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

  const cv::Mat bgr_image(color_image_->height, color_image_->width, CV_8UC3,
                          color_image_->data.data(), color_image_->step);
  cv::Mat rgb_image(color_image_->height, color_image_->width, CV_8UC3,
                    color_snapshot.image.data.data(), color_image_->step);
  cv::cvtColor(bgr_image, rgb_image, cv::COLOR_BGR2RGB);

  response->images.push_back(std::move(color_snapshot));

  response->success = true;
}

}  // namespace flowstate_luxonis
