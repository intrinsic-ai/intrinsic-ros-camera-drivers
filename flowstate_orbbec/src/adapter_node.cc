#include "flowstate_orbbec/adapter_node.h"

#include <memory>

#include "opencv2/core.hpp"
#include "orbbec_camera/ob_camera_node_driver.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "snapshot_interfaces/msg/image_snapshot.hpp"

#define SEND_DEPTH 0

namespace flowstate_orbbec {

using snapshot_interfaces::srv::Describe;
using snapshot_interfaces::srv::Snapshot;

AdapterNode::AdapterNode(const std::string& serial,
                         const std::string& ip_address)
    : Node(std::string("orbbec_") + serial),
      serial_(serial),
      ip_address_(ip_address) {
  const std::string orbbec_node_name = std::string("orbbec_camera_node");
  const std::string orbbec_ns = std::string("orbbec/camera_") + serial;
  rclcpp::NodeOptions orbbec_node_options =
      rclcpp::NodeOptions()
          .append_parameter_override(rclcpp::Parameter("serial_number", serial))
          .append_parameter_override(
              rclcpp::Parameter("enumerate_net_device", true))
          .append_parameter_override(rclcpp::Parameter("enable_depth", false))
          .append_parameter_override(rclcpp::Parameter("color_fps", 10))
          .append_parameter_override(rclcpp::Parameter("color_format", "RGB"))
          .append_parameter_override(rclcpp::Parameter("color_width", 1280))
          .append_parameter_override(rclcpp::Parameter("color_height", 800))
          .append_parameter_override(rclcpp::Parameter("color_sharpness", 75))
          .append_parameter_override(rclcpp::Parameter("enable_color", true))
#if SEND_DEPTH
          .append_parameter_override(rclcpp::Parameter("depth_fps", 10))
          .append_parameter_override(rclcpp::Parameter("right_ir_fps", 10))
          .append_parameter_override(rclcpp::Parameter("enable_depth", true))
#else
          .append_parameter_override(rclcpp::Parameter("enable_depth", false))
#endif
          .append_parameter_override(rclcpp::Parameter("left_ir_fps", 10))
          .append_parameter_override(rclcpp::Parameter("left_ir_format", "Y8"))
          .append_parameter_override(rclcpp::Parameter("left_ir_width", 1280))
          .append_parameter_override(rclcpp::Parameter("left_ir_height", 800))
          .append_parameter_override(rclcpp::Parameter("enable_left_ir", true));
  orbbec_node_ = std::make_unique<orbbec_camera::OBCameraNodeDriver>(
      orbbec_node_name, orbbec_ns, orbbec_node_options);
  color_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      absl::StrFormat("orbbec/camera_%s/color/camera_info", serial_), 2,
      [this](sensor_msgs::msg::CameraInfo::UniquePtr msg) {
        absl::MutexLock lock(&this->camera_info_mutex_);
        this->color_camera_info_ = std::move(msg);
      });
  ir_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      absl::StrFormat("orbbec/camera_%s/left_ir/camera_info", serial_), 2,
      [this](sensor_msgs::msg::CameraInfo::UniquePtr msg) {
        absl::MutexLock lock(&this->camera_info_mutex_);
        this->ir_camera_info_ = std::move(msg);
      });
#if SEND_DEPTH
  depth_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      absl::StrFormat("orbbec/camera_%s/depth/camera_info", serial_), 2,
      [this](sensor_msgs::msg::CameraInfo::UniquePtr msg) {
        absl::MutexLock lock(&this->camera_info_mutex_);
        this->depth_camera_info_ = std::move(msg);
      });
#endif
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
  ir_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      IrImageTopic(), 2, [this](sensor_msgs::msg::Image::UniquePtr msg) {
        absl::MutexLock lock(&this->image_mutex_);
        this->ir_image_ = std::move(msg);
      });
#if SEND_DEPTH
  depth_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      DepthImageTopic(), 2, [this](sensor_msgs::msg::Image::UniquePtr msg) {
        absl::MutexLock lock(&this->image_mutex_);
        this->depth_image_ = std::move(msg);
      });
#endif

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
    RCLCPP_INFO(this->get_logger(), "Destroying orbbec_camera_node...");
    orbbec_node_.reset();
    rclcpp::sleep_for(std::chrono::milliseconds(500));  // maybe this helps?
    RCLCPP_INFO(this->get_logger(), "Done destroying orbbec_camera_node");
    exited_thread_ = true;
  });
}

std::string AdapterNode::ColorImageTopic() const {
  return absl::StrFormat("/orbbec/camera_%s/color/image_raw", serial_);
}

std::string AdapterNode::IrImageTopic() const {
  return absl::StrFormat("/orbbec/camera_%s/left_ir/image_raw", serial_);
}

std::string AdapterNode::DepthImageTopic() const {
  return absl::StrFormat("/orbbec/camera_%s/depth/image_raw", serial_);
}

absl::Status AdapterNode::Main() {
  RCLCPP_INFO(get_logger(), "AdapterNode::Main()");
  t_last_color_image_ = get_clock()->now();
  rclcpp::executors::SingleThreadedExecutor executor;
  liveness_timer_ =
      create_wall_timer(std::chrono::seconds(1), [this, &executor]() {
        absl::MutexLock timeout_lock(&timeout_mutex_);
        if ((get_clock()->now() - t_last_color_image_).seconds() > 20.0) {
          RCLCPP_ERROR(get_logger(), "No new image arrived for 20 seconds");
          executor.cancel();
        }
      });

  executor.add_node(this->get_node_base_interface());
  executor.add_node(orbbec_node_->get_node_base_interface());
  executor.spin();
  return absl::OkStatus();
}

void AdapterNode::DescribeCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<snapshot_interfaces::srv::Describe::Request>,
    const std::shared_ptr<snapshot_interfaces::srv::Describe::Response>
        response) {
  absl::MutexLock lock(&camera_info_mutex_);
  if (!color_camera_info_ || !ir_camera_info_) {
    response->error_message = "CameraInfo not yet received from camera";
    response->success = false;
    RCLCPP_ERROR(get_logger(), response->error_message.c_str());
    return;
  }

  snapshot_interfaces::msg::SensorInfo color_info;
  color_info.sensor_name = "rgb";
  color_info.topic_name = ColorImageTopic();
  color_info.sensor_type = snapshot_interfaces::msg::SensorInfo::IMAGE;
  color_info.camera_t_sensor.transform.rotation.w = 1.0;  // todo: get static transform
  color_info.info.push_back(*color_camera_info_);
  response->sensors.push_back(color_info);

  snapshot_interfaces::msg::SensorInfo ir_info;
  ir_info.sensor_name = "ir_left";
  ir_info.topic_name = IrImageTopic();
  ir_info.sensor_type = snapshot_interfaces::msg::SensorInfo::IMAGE;
  ir_info.camera_t_sensor.transform.rotation.w = 1.0;  // todo: get static transform
  ir_info.info.push_back(*ir_camera_info_);
  response->sensors.push_back(ir_info);

#if SEND_DEPTH
  snapshot_interfaces::msg::SensorInfo depth_info;
  depth_info.sensor_name = "depth";
  depth_info.topic_name = DepthImageTopic();
  depth_info.sensor_type = snapshot_interfaces::msg::SensorInfo::IMAGE;
  depth_info.camera_t_sensor.transform.rotation.w = 1.0;  // todo: get static transform
  depth_info.info.push_back(*depth_camera_info_);
  response->sensors.push_back(depth_info);
#endif

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
  snapshot_interfaces::msg::ImageSnapshot ir_snapshot;
#if SEND_DEPTH
  snapshot_interfaces::msg::ImageSnapshot depth_snapshot;
#endif

  color_snapshot.topic_name = ColorImageTopic();
  ir_snapshot.topic_name = IrImageTopic();
#if SEND_DEPTH
  depth_snapshot.topic_name = DepthImageTopic();
#endif

  // Lock and copy the most recent CameraInfo messages
  {
    absl::MutexLock lock(&camera_info_mutex_);
    if (!color_camera_info_ || !ir_camera_info_) {
      response->error_message = "CameraInfo not yet received";
      response->success = false;
      RCLCPP_ERROR(get_logger(), response->error_message.c_str());
      return;
    }
    color_snapshot.camera_info = *color_camera_info_;
    ir_snapshot.camera_info = *ir_camera_info_;
#if SEND_DEPTH
    if (depth_camera_info_) {
      depth_snapshot.camera_info = *depth_camera_info_;
    }
#endif
  }

  // Lock and copy the most recent Image messages
  absl::MutexLock lock(&image_mutex_);
  if (!color_image_ || !ir_image_) {
    response->error_message = "images not yet received from camera";
    response->success = false;
    RCLCPP_ERROR(get_logger(), response->error_message.c_str());
    return;
  }

  color_snapshot.image = *color_image_;
  response->images.push_back(std::move(color_snapshot));

  ir_snapshot.image = *ir_image_;
  response->images.push_back(std::move(ir_snapshot));

#if SEND_DEPTH
  if (!depth_image_) {
    response->error_message = "depth image not yet received from camera";
    response->success = false;
    RCLCPP_ERROR(get_logger(), response->error_message.c_str());
    return;
  }
  // The Orbbec camera returns the depth image as 16-bit images in millimeters.
  // We want to convert that to 32-bit float (meters) for Flowstate.
  depth_snapshot.image.header = depth_image_->header;
  depth_snapshot.image.height = depth_image_->height;
  depth_snapshot.image.width = depth_image_->width;
  depth_snapshot.image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  depth_snapshot.image.is_bigendian = false;
  depth_snapshot.image.step = 4 * depth_snapshot.image.width;
  depth_snapshot.image.data.resize(depth_snapshot.image.step *
                                   depth_snapshot.image.height);
  // Use OpenCV's amazingly optimized implementation to do the conversion
  const cv::Mat depth_unsigned(depth_image_->height, depth_image_->width,
                               CV_16U, depth_image_->data.data());
  cv::Mat depth_float(depth_snapshot.image.height, depth_snapshot.image.width,
                      CV_32F, depth_snapshot.image.data.data());
  depth_unsigned.convertTo(depth_float, CV_32F, 0.001);

  response->images.push_back(std::move(depth_snapshot));
#endif

  response->success = true;
}

}  // namespace flowstate_orbbec
