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

#ifndef FLOWSTATE_ZED_ZED_PARAMETERS_H_
#define FLOWSTATE_ZED_ZED_PARAMETERS_H_

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "flowstate_zed/camera_discovery.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

namespace flowstate_zed {

// Like all its parameters, the wrapper declares this one in initNode(), not in
// its constructor.
inline constexpr absl::string_view kWrapperFrameRate =
    "general.grab_frame_rate";

inline constexpr int64_t kExposureMinUs = 28;
inline constexpr int64_t kExposureMaxUs = 30000;
inline constexpr double kExposureMinS = kExposureMinUs * 1e-6;
inline constexpr double kExposureMaxS = kExposureMaxUs * 1e-6;
inline constexpr int64_t kGainMinMdb = 1000;
inline constexpr int64_t kGainMaxMdb = 16000;
inline constexpr double kGainMinDb = kGainMinMdb * 1e-3;
inline constexpr double kGainMaxDb = kGainMaxMdb * 1e-3;
inline constexpr double kGainDefaultDb = kGainMinDb;
inline constexpr double kFps15 = 15.0;
inline constexpr double kFps30 = 30.0;
inline constexpr double kFps60 = 60.0;
inline constexpr double kFpsStep = 15.0;
inline constexpr double kFpsDefault = kFps30;
inline constexpr int64_t kExposureCompensationMin = 0;
inline constexpr int64_t kExposureCompensationMax = 100;
inline constexpr int64_t kExposureCompensationDefault = 50;
inline constexpr int64_t kAeAntibandingMin = 0;
inline constexpr int64_t kAeAntibandingMax = 3;
inline constexpr int64_t kAeAntibandingDefault = 1;
inline constexpr int64_t kWhiteBalanceMinK = 2800;
inline constexpr int64_t kWhiteBalanceMaxK = 6500;
inline constexpr int64_t kWhiteBalanceStepK = 100;
inline constexpr int64_t kWhiteBalanceDefaultK = 4200;
// The wrapper takes video.whitebalance_temperature in 100 K units (28..65).
inline constexpr int64_t kWhiteBalanceWrapperUnitK = 100;
inline constexpr int64_t kGammaMin = 1;
inline constexpr int64_t kGammaMax = 9;
inline constexpr int64_t kGammaDefault = 8;
inline constexpr int64_t kSaturationMin = 0;
inline constexpr int64_t kSaturationMax = 8;
inline constexpr int64_t kSaturationDefault = 4;
inline constexpr int64_t kSharpnessMin = 0;
inline constexpr int64_t kSharpnessMax = 8;
inline constexpr int64_t kSharpnessDefault = 4;
inline constexpr int64_t kDenoisingMin = 0;
inline constexpr int64_t kDenoisingMax = 100;
inline constexpr int64_t kDenoisingDefault = 50;

// Digital gain stays at 1 (no ISP amplification) in both automatic and manual
// modes, matching common_mono.yaml and zedx_ar0234.isp. In automatic mode,
// analog gain is capped at 1.5 dB (1500 mdB) so the Argus AEC/AGC loop uses
// shutter time up to 30 ms before raising sensor gain.
inline constexpr int64_t kManualDigitalGain = 1;
inline constexpr int64_t kAutoGainMaxMdb = 1500;

// Atomic snapshot of the adapter's ROS parameters, already converted to the
// wrapper's units where they differ.
struct CameraSettings {
  bool auto_exposure = true;
  int64_t exposure_us = kExposureMaxUs;
  int64_t gain_mdb = kGainMinMdb;
  int64_t fps = static_cast<int64_t>(kFpsDefault);
  int64_t exposure_compensation = kExposureCompensationDefault;
  int64_t ae_antibanding = kAeAntibandingDefault;
  bool auto_white_balance = true;
  int64_t white_balance_100k =
      kWhiteBalanceDefaultK / kWhiteBalanceWrapperUnitK;
  int64_t gamma = kGammaDefault;
  int64_t saturation = kSaturationDefault;
  int64_t sharpness = kSharpnessDefault;
  int64_t denoising = kDenoisingDefault;
};

// Tracks stereo (ZedCamera) auto-exposure range alternation. ZedCamera skips
// setCameraSettings() when getCameraSettings(AUTO_*_RANGE) equals the target
// range, which happens when returning to auto after locking manual [x, x].
// Alternating the upper bound by 1 unit on each manual -> auto transition
// forces the SDK write.
struct StereoAutoRangeState {
  bool last_auto_exposure = true;
  bool trim_auto_range_max = false;

  bool Advance(bool auto_exposure);
};

int64_t SecondsToMicroseconds(double seconds);
int64_t DecibelsToMillidecibels(double decibels);

bool HasParameter(absl::Span<const rclcpp::Parameter> parameters,
                  absl::string_view name);

void DeclareAdapterParameters(rclcpp::Node& node);

CameraSettings ReadCameraSettings(const rclcpp::Node& node);

rcl_interfaces::msg::SetParametersResult ValidateCameraParameters(
    absl::Span<const rclcpp::Parameter> parameters);

bool RequiresWrapperRestart(absl::Span<const rclcpp::Parameter> changed,
                            int64_t running_fps, int64_t target_fps);

std::vector<rclcpp::Parameter> BuildWrapperVideoParameters(
    const CameraSettings& settings, bool stereo,
    bool trim_auto_range_max = false);

// Filters `desired` to parameters whose values differ from `get_current(name)`.
std::vector<rclcpp::Parameter> DiffWrapperParameters(
    absl::Span<const rclcpp::Parameter> desired,
    const std::function<rclcpp::ParameterValue(const std::string&)>&
        get_current);

rclcpp::NodeOptions BuildWrapperOptions(const DiscoveredCamera& camera,
                                        const CameraSettings& settings,
                                        bool trim_auto_range_max = false);

}  // namespace flowstate_zed

#endif  // FLOWSTATE_ZED_ZED_PARAMETERS_H_
