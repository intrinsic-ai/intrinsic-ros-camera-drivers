#include "flowstate_orbbec/orbbec_startup_parameters.h"

#include <string>

#include "absl/status/status.h"
#include "rclcpp/rclcpp.hpp"

namespace flowstate_orbbec {

rclcpp::NodeOptions OrbbecStartupParameters::CreateNodeOptions(
    const std::string& serial) {
  const std::string camera_name = std::string("orbbec_") + serial;

  rclcpp::NodeOptions options =
      rclcpp::NodeOptions()
          .append_parameter_override(
              rclcpp::Parameter("camera_name", camera_name))
          .append_parameter_override(rclcpp::Parameter("serial_number", serial))
          .append_parameter_override(
              rclcpp::Parameter("enumerate_net_device", true));

  if (enable_rgb_) {
    options =
        options.append_parameter_override(rclcpp::Parameter("color_fps", fps_))
            .append_parameter_override(rclcpp::Parameter("color_format", "RGB"))
            .append_parameter_override(rclcpp::Parameter("color_width", 1280))
            .append_parameter_override(rclcpp::Parameter("color_height", 800))
            .append_parameter_override(rclcpp::Parameter("color_sharpness", 75))
            .append_parameter_override(rclcpp::Parameter("enable_color", true));
  } else {
    options = options.append_parameter_override(
        rclcpp::Parameter("enable_color", false));
  }

  if (enable_depth_) {
    options =
        options.append_parameter_override(rclcpp::Parameter("depth_fps", 5))
            .append_parameter_override(rclcpp::Parameter("enable_depth", true));
  } else {
    options = options.append_parameter_override(
        rclcpp::Parameter("enable_depth", false));
  }

  if (enable_left_ir_) {
    options =
        options.append_parameter_override(rclcpp::Parameter("left_ir_fps", 5))
            .append_parameter_override(
                rclcpp::Parameter("left_ir_format", "Y8"))
            .append_parameter_override(rclcpp::Parameter("left_ir_width", 1280))
            .append_parameter_override(rclcpp::Parameter("left_ir_height", 800))
            .append_parameter_override(
                rclcpp::Parameter("enable_left_ir", true));
  } else {
    options = options.append_parameter_override(
        rclcpp::Parameter("enable_left_ir", false));
  }

  if (enable_right_ir_) {
    options =
        options.append_parameter_override(rclcpp::Parameter("right_ir_fps", 5))
            .append_parameter_override(
                rclcpp::Parameter("right_ir_format", "Y8"))
            .append_parameter_override(
                rclcpp::Parameter("right_ir_width", 1280))
            .append_parameter_override(
                rclcpp::Parameter("right_ir_height", 800))
            .append_parameter_override(
                rclcpp::Parameter("enable_right_ir", true));
  } else {
    options = options.append_parameter_override(
        rclcpp::Parameter("enable_right_ir", false));
  }

  return options;
}

absl::Status OrbbecStartupParameters::SetFps(const int fps) {
  if (fps != 5 && fps != 10) {
    return absl::InvalidArgumentError("FPS must be either 5 or 10");
  }
  if (fps_ == fps) {
    return absl::OkStatus();
  }
  fps_ = fps;
  has_changed_ = true;
  return absl::OkStatus();
}

void OrbbecStartupParameters::EnableRgb(const bool enable) {
  if (enable_rgb_ == enable) {
    return;
  }
  enable_rgb_ = enable;
  has_changed_ = true;
}

void OrbbecStartupParameters::EnableLeftIr(const bool enable) {
  if (enable_left_ir_ == enable) {
    return;
  }
  enable_left_ir_ = enable;
  has_changed_ = true;
}

void OrbbecStartupParameters::EnableRightIr(const bool enable) {
  if (enable_right_ir_ == enable) {
    return;
  }
  enable_right_ir_ = enable;
  has_changed_ = true;
}

void OrbbecStartupParameters::EnableDepth(const bool enable) {
  if (enable_depth_ == enable) {
    return;
  }
  enable_depth_ = enable;
  has_changed_ = true;
}

}  // namespace flowstate_orbbec
