// Copyright 2026 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "flowstate_zed/zed_parameters.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace flowstate_zed {
namespace {

rcl_interfaces::msg::ParameterDescriptor FloatDescriptor(
    double from_value, double to_value, double step,
    absl::string_view description) {
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = from_value;
  range.to_value = to_value;
  range.step = step;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = std::string(description);
  descriptor.floating_point_range.push_back(range);
  return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor IntegerDescriptor(
    int64_t from_value, int64_t to_value, int64_t step,
    absl::string_view description) {
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = from_value;
  range.to_value = to_value;
  range.step = step;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = std::string(description);
  descriptor.integer_range.push_back(range);
  return descriptor;
}

// Mono (ZedCameraOne): digital gain stays pinned at 1; automatic mode bounds
// exposure to [28, 30000] us and analog gain to [1000, kAutoGainMaxMdb] mdB.
// patches/zed-ros2-wrapper.patch fixes ZedCameraOne so auto -> manual writes
// equal values cleanly and auto range overrides are not reset to 16000 mdB.
std::vector<rclcpp::Parameter> MonoExposureParameters(bool auto_exposure,
                                                      int64_t exposure_us,
                                                      int64_t gain_mdb) {
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("video.auto_digital_gain", false);
  parameters.emplace_back("video.digital_gain", kManualDigitalGain);
  parameters.emplace_back("video.auto_digital_gain_range_min",
                          kManualDigitalGain);
  parameters.emplace_back("video.auto_digital_gain_range_max",
                          kManualDigitalGain);
  if (auto_exposure) {
    parameters.emplace_back("video.auto_exposure", true);
    parameters.emplace_back("video.auto_exposure_time_range_min",
                            kExposureMinUs);
    parameters.emplace_back("video.auto_exposure_time_range_max",
                            kExposureMaxUs);
    parameters.emplace_back("video.auto_analog_gain", true);
    parameters.emplace_back("video.auto_analog_gain_range_min", kGainMinMdb);
    parameters.emplace_back("video.auto_analog_gain_range_max",
                            kAutoGainMaxMdb);
  } else {
    parameters.emplace_back("video.auto_exposure", false);
    parameters.emplace_back("video.exposure_time", exposure_us);
    parameters.emplace_back("video.auto_analog_gain", false);
    parameters.emplace_back("video.analog_gain", gain_mdb);
  }
  return parameters;
}

// Stereo (ZedCamera): the wrapper's automatic exposure stays on and
// video.auto_exposure_gain is never touched. A manual value is a collapsed
// [x, x] range; automatic uses [28, 30000] us and [1000, kAutoGainMaxMdb] mdB
// with digital gain pinned at 1.
std::vector<rclcpp::Parameter> StereoExposureParameters(
    bool auto_exposure, int64_t exposure_us, int64_t gain_mdb,
    bool trim_auto_range_max) {
  int64_t exposure_min = exposure_us;
  int64_t exposure_max = exposure_us;
  int64_t gain_min = gain_mdb;
  int64_t gain_max = gain_mdb;
  if (auto_exposure) {
    const int64_t trim = trim_auto_range_max ? 1 : 0;
    exposure_min = kExposureMinUs;
    exposure_max = kExposureMaxUs - trim;
    gain_min = kGainMinMdb;
    gain_max = kAutoGainMaxMdb - trim;
  }
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("video.auto_exposure_time_range_min", exposure_min);
  parameters.emplace_back("video.auto_exposure_time_range_max", exposure_max);
  parameters.emplace_back("video.auto_analog_gain_range_min", gain_min);
  parameters.emplace_back("video.auto_analog_gain_range_max", gain_max);
  parameters.emplace_back("video.auto_digital_gain_range_min",
                          kManualDigitalGain);
  parameters.emplace_back("video.auto_digital_gain_range_max",
                          kManualDigitalGain);
  return parameters;
}

}  // namespace

bool StereoAutoRangeState::Advance(bool auto_exposure) {
  if (!last_auto_exposure && auto_exposure) {
    trim_auto_range_max = !trim_auto_range_max;
  }
  last_auto_exposure = auto_exposure;
  return trim_auto_range_max;
}

int64_t SecondsToMicroseconds(double seconds) {
  return std::llround(seconds * 1e6);
}

int64_t DecibelsToMillidecibels(double decibels) {
  return std::llround(decibels * 1e3);
}

bool HasParameter(absl::Span<const rclcpp::Parameter> parameters,
                  absl::string_view name) {
  return std::ranges::any_of(parameters, [name](const rclcpp::Parameter& p) {
    return p.get_name() == name;
  });
}

void DeclareAdapterParameters(rclcpp::Node& node) {
  rcl_interfaces::msg::ParameterDescriptor auto_exposure;
  auto_exposure.description =
      "Automatic exposure (0.000028 to 0.030 s) and analog gain (1.0 to "
      "1.5 dB).";
  node.declare_parameter("auto_exposure", true, auto_exposure);

  node.declare_parameter(
      "exposure", kExposureMaxS,
      FloatDescriptor(
          kExposureMinS, kExposureMaxS, 0.0,
          "Exposure time in seconds. Effective when auto_exposure is false."));
  node.declare_parameter(
      "gain", kGainDefaultDb,
      FloatDescriptor(
          kGainMinDb, kGainMaxDb, 0.0,
          "Analog gain in dB. Effective when auto_exposure is false."));

  // Declared as double to match Flowstate's float setting schema for "fps".
  node.declare_parameter(
      "fps", kFpsDefault,
      FloatDescriptor(
          kFps15, kFps60, kFpsStep,
          "Frame rate: 15, 30 or 60. A change restarts the camera."));

  node.declare_parameter(
      "exposure_compensation", kExposureCompensationDefault,
      IntegerDescriptor(kExposureCompensationMin, kExposureCompensationMax, 1,
                        "Exposure compensation (50 is neutral)."));
  node.declare_parameter(
      "ae_antibanding", kAeAntibandingDefault,
      IntegerDescriptor(
          kAeAntibandingMin, kAeAntibandingMax, 1,
          "Auto-exposure anti-banding: 0 (off), 1 (auto), 2 (50 Hz), "
          "3 (60 Hz). Effective when auto_exposure is true."));

  rcl_interfaces::msg::ParameterDescriptor auto_white_balance;
  auto_white_balance.description = "Automatic white balance.";
  node.declare_parameter("auto_white_balance", true, auto_white_balance);
  node.declare_parameter(
      "white_balance", kWhiteBalanceDefaultK,
      IntegerDescriptor(kWhiteBalanceMinK, kWhiteBalanceMaxK,
                        kWhiteBalanceStepK,
                        "White balance color temperature in K. Effective when "
                        "auto_white_balance is false."));

  node.declare_parameter(
      "gamma", kGammaDefault,
      IntegerDescriptor(kGammaMin, kGammaMax, 1, "ISP gamma correction."));
  node.declare_parameter("saturation", kSaturationDefault,
                         IntegerDescriptor(kSaturationMin, kSaturationMax, 1,
                                           "Color saturation."));
  node.declare_parameter(
      "sharpness", kSharpnessDefault,
      IntegerDescriptor(kSharpnessMin, kSharpnessMax, 1, "Image sharpness."));
  node.declare_parameter("denoising", kDenoisingDefault,
                         IntegerDescriptor(kDenoisingMin, kDenoisingMax, 1,
                                           "ISP denoising strength."));
}

CameraSettings ReadCameraSettings(const rclcpp::Node& node) {
  // Single get_parameters() call snapshots all settings atomically under one
  // NodeParameters lock, preventing tearing across related parameters.
  const std::vector<rclcpp::Parameter> params = node.get_parameters({
      "auto_exposure",
      "exposure",
      "gain",
      "fps",
      "exposure_compensation",
      "ae_antibanding",
      "auto_white_balance",
      "white_balance",
      "gamma",
      "saturation",
      "sharpness",
      "denoising",
  });
  return {
      .auto_exposure = params[0].as_bool(),
      .exposure_us = SecondsToMicroseconds(params[1].as_double()),
      .gain_mdb = DecibelsToMillidecibels(params[2].as_double()),
      .fps = std::llround(params[3].as_double()),
      .exposure_compensation = params[4].as_int(),
      .ae_antibanding = params[5].as_int(),
      .auto_white_balance = params[6].as_bool(),
      .white_balance_100k = params[7].as_int() / kWhiteBalanceWrapperUnitK,
      .gamma = params[8].as_int(),
      .saturation = params[9].as_int(),
      .sharpness = params[10].as_int(),
      .denoising = params[11].as_int(),
  };
}

rcl_interfaces::msg::SetParametersResult ValidateCameraParameters(
    absl::Span<const rclcpp::Parameter> parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const rclcpp::Parameter& parameter : parameters) {
    const std::string& name = parameter.get_name();
    if (name == "fps") {
      const double fps = parameter.as_double();
      // FloatingPointRange [15, 60] step 15 also permits 45, which HD1200 does
      // not support; reject non-finite and unsupported values here.
      if (!std::isfinite(fps) ||
          (fps != kFps15 && fps != kFps30 && fps != kFps60)) {
        result.successful = false;
        result.reason = "fps must be 15, 30 or 60";
        return result;
      }
    } else if ((name == "exposure" || name == "gain") &&
               !std::isfinite(parameter.as_double())) {
      // rclcpp's FloatingPointRange check does not reject NaN.
      result.successful = false;
      result.reason = absl::StrCat(name, " must be finite");
      return result;
    }
  }
  return result;
}

bool RequiresWrapperRestart(absl::Span<const rclcpp::Parameter> changed,
                            int64_t running_fps, int64_t target_fps) {
  return HasParameter(changed, "fps") && running_fps != target_fps;
}

std::vector<rclcpp::Parameter> BuildWrapperVideoParameters(
    const CameraSettings& settings, bool stereo, bool trim_auto_range_max) {
  std::vector<rclcpp::Parameter> parameters =
      stereo ? StereoExposureParameters(settings.auto_exposure,
                                        settings.exposure_us, settings.gain_mdb,
                                        trim_auto_range_max)
             : MonoExposureParameters(settings.auto_exposure,
                                      settings.exposure_us, settings.gain_mdb);

  parameters.emplace_back("video.exposure_compensation",
                          settings.exposure_compensation);
  parameters.emplace_back("video.ae_antibanding", settings.ae_antibanding);

  if (settings.auto_white_balance) {
    parameters.emplace_back("video.auto_whitebalance", true);
  } else {
    parameters.emplace_back("video.auto_whitebalance", false);
    parameters.emplace_back("video.whitebalance_temperature",
                            settings.white_balance_100k);
  }

  parameters.emplace_back("video.gamma", settings.gamma);
  parameters.emplace_back("video.saturation", settings.saturation);
  parameters.emplace_back("video.sharpness", settings.sharpness);
  parameters.emplace_back("video.denoising", settings.denoising);
  return parameters;
}

std::vector<rclcpp::Parameter> DiffWrapperParameters(
    absl::Span<const rclcpp::Parameter> desired,
    const std::function<rclcpp::ParameterValue(const std::string&)>&
        get_current) {
  std::vector<rclcpp::Parameter> changes;
  for (const rclcpp::Parameter& parameter : desired) {
    if (get_current(parameter.get_name()) != parameter.get_parameter_value()) {
      changes.push_back(parameter);
    }
  }
  return changes;
}

rclcpp::NodeOptions BuildWrapperOptions(const DiscoveredCamera& camera,
                                        const CameraSettings& settings,
                                        bool trim_auto_range_max) {
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true)
      .arguments({"--ros-args", "-r",
                  absl::StrCat("__ns:=/zed/camera_", camera.serial), "-r",
                  absl::StrCat("__node:=", kWrapperNodeName)})
      .append_parameter_override("general.camera_model", camera.wrapper_model)
      .append_parameter_override("general.camera_name",
                                 absl::StrCat("zed_", camera.serial))
      .append_parameter_override("general.serial_number",
                                 static_cast<int64_t>(camera.serial_number))
      .append_parameter_override("general.grab_resolution", "HD1200")
      .append_parameter_override("general.pub_resolution", "NATIVE")
      .append_parameter_override(std::string(kWrapperFrameRate),
                                 static_cast<int>(settings.fps))
      .append_parameter_override("video.publish_raw", true)
      .append_parameter_override("video.enable_24bit_output", true)
      .append_parameter_override("sensors.publish_imu", false)
      .append_parameter_override("general.publish_status", false);
  if (camera.stereo) {
    options.append_parameter_override("video.publish_left_right", true)
        .append_parameter_override("video.publish_rgb", false)
        .append_parameter_override("depth.depth_mode", "NONE")
        .append_parameter_override("depth.publish_depth_map", false)
        .append_parameter_override("depth.publish_point_cloud", false)
        .append_parameter_override("pos_tracking.pos_tracking_enabled", false)
        // Matches ReadRightSensorPose().
        .append_parameter_override("general.self_calib", false)
        // 0: publish every frame. The wrapper's own default is 15 Hz at any
        // grab rate.
        .append_parameter_override("general.pub_frame_rate", 0.0);
  }
  for (const rclcpp::Parameter& parameter : BuildWrapperVideoParameters(
           settings, camera.stereo, trim_auto_range_max)) {
    options.append_parameter_override(parameter.get_name(),
                                      parameter.get_parameter_value());
  }
  return options;
}

}  // namespace flowstate_zed
