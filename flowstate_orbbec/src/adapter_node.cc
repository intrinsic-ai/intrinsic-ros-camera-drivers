#include "flowstate_orbbec/adapter_node.h"

#include <memory>

#include "orbbec_camera/ob_camera_node_driver.h"
#include "rclcpp/rclcpp.hpp"

namespace flowstate_orbbec {

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
          .append_parameter_override(rclcpp::Parameter("color_format", "RGB"))
          .append_parameter_override(rclcpp::Parameter("color_width", 1280))
          .append_parameter_override(rclcpp::Parameter("color_height", 800))
          .append_parameter_override(rclcpp::Parameter("enable_color", true))
          .append_parameter_override(rclcpp::Parameter("left_ir_format", "Y8"))
          .append_parameter_override(rclcpp::Parameter("left_ir_width", 1280))
          .append_parameter_override(rclcpp::Parameter("left_ir_height", 800))
          .append_parameter_override(rclcpp::Parameter("enable_left_ir", true));
  orbbec_node_ = std::make_unique<orbbec_camera::OBCameraNodeDriver>(
      orbbec_node_name, orbbec_ns, orbbec_node_options);
  thread_ = std::thread([this]() {
    const absl::Status status = this->main();
    if (!status.ok()) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "node thread error: " << status);
    } else {
      RCLCPP_INFO(this->get_logger(), "exited thread");
    }
    RCLCPP_INFO(this->get_logger(), "Destroying orbbec_camera_node...");
    orbbec_node_.reset();
    rclcpp::sleep_for(std::chrono::milliseconds(500));  // maybe this helps?
    RCLCPP_INFO(this->get_logger(), "Done destroying orbbec_camera_node");
    exited_thread_ = true;
  });
}

absl::Status AdapterNode::main() {
  RCLCPP_INFO(get_logger(), "AdapterNode::main()");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(this->get_node_base_interface());
  executor.add_node(orbbec_node_->get_node_base_interface());
  // TODO: add some other test for camera health, to exit this loop if it's bad
  while (rclcpp::ok()) {
    executor.spin_some();
    // maybe do something
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  return absl::OkStatus();
}

}  // namespace flowstate_orbbec

