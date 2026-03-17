#ifndef FLOWSTATE_ORBBEC_STARTUP_PARAMETERS_H_
#define FLOWSTATE_ORBBEC_STARTUP_PARAMETERS_H_

#include "absl/status/status.h"
#include "rclcpp/rclcpp.hpp"

namespace flowstate_orbbec {

class OrbbecStartupParameters {
 public:
  OrbbecStartupParameters() {}
  rclcpp::NodeOptions CreateNodeOptions(const std::string& serial);

  absl::Status SetFps(const int fps);
  void EnableRgb(const bool enable);
  void EnableLeftIr(const bool enable);
  void EnableRightIr(const bool enable);
  void EnableDepth(const bool enable);

  bool IsRgbEnabled() const { return enable_rgb_; }
  bool IsLeftIrEnabled() const { return enable_left_ir_; }
  bool IsRightIrEnabled() const { return enable_right_ir_; }
  bool IsDepthEnabled() const { return enable_depth_; }
  bool HasChanged() const { return has_changed_; }
  void ResetHasChanged() { has_changed_ = false; }
 
 private:
  bool has_changed_ = false;
  int fps_ = 5;
  bool enable_rgb_ = true;
  bool enable_left_ir_ = true;
  bool enable_right_ir_ = true;
  bool enable_depth_ = true;
};

}  // namespace flowstate_orbbec

#endif  // FLOWSTATE_ORBBEC_STARTUP_PARAMETERS_H_
