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

#include <gtest/gtest.h>

#include <limits>
#include <map>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace flowstate_zed {
namespace {

std::map<std::string, rclcpp::ParameterValue> ToMap(
    const std::vector<rclcpp::Parameter>& params) {
  std::map<std::string, rclcpp::ParameterValue> out;
  for (const rclcpp::Parameter& p : params) {
    out[p.get_name()] = p.get_parameter_value();
  }
  return out;
}

TEST(ZedParametersTest, UnitConversions) {
  EXPECT_EQ(SecondsToMicroseconds(kExposureMinS), kExposureMinUs);
  EXPECT_EQ(SecondsToMicroseconds(kExposureMaxS), kExposureMaxUs);
  EXPECT_EQ(DecibelsToMillidecibels(kGainMinDb), kGainMinMdb);
  EXPECT_EQ(DecibelsToMillidecibels(1.5), kAutoGainMaxMdb);
  EXPECT_EQ(DecibelsToMillidecibels(kGainMaxDb), kGainMaxMdb);
}

TEST(ZedParametersTest, ValidateCameraParametersChecksFpsAndFiniteness) {
  EXPECT_TRUE(
      ValidateCameraParameters({rclcpp::Parameter("fps", kFps15)}).successful);
  EXPECT_TRUE(
      ValidateCameraParameters({rclcpp::Parameter("fps", kFps30)}).successful);
  EXPECT_TRUE(
      ValidateCameraParameters({rclcpp::Parameter("fps", kFps60)}).successful);
  EXPECT_FALSE(
      ValidateCameraParameters({rclcpp::Parameter("fps", 45.0)}).successful);
  EXPECT_FALSE(
      ValidateCameraParameters({rclcpp::Parameter("fps", 10.0)}).successful);

  const double kNaN = std::numeric_limits<double>::quiet_NaN();
  const double kInf = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(
      ValidateCameraParameters({rclcpp::Parameter("fps", kNaN)}).successful);
  EXPECT_FALSE(ValidateCameraParameters({rclcpp::Parameter("exposure", kNaN)})
                   .successful);
  EXPECT_FALSE(
      ValidateCameraParameters({rclcpp::Parameter("gain", kInf)}).successful);
  EXPECT_TRUE(ValidateCameraParameters({rclcpp::Parameter("exposure", 0.015),
                                        rclcpp::Parameter("gain", 2.5)})
                  .successful);
}

TEST(ZedParametersTest, RequiresWrapperRestartOnlyOnFpsChange) {
  EXPECT_TRUE(
      RequiresWrapperRestart({rclcpp::Parameter("fps", kFps60)}, 30, 60));
  EXPECT_FALSE(
      RequiresWrapperRestart({rclcpp::Parameter("fps", kFps30)}, 30, 30));
  EXPECT_FALSE(
      RequiresWrapperRestart({rclcpp::Parameter("denoising", 80)}, 30, 30));
}

TEST(ZedParametersTest, StereoAutoRangeStateFlipsOnlyOnManualToAuto) {
  StereoAutoRangeState state;
  // Idempotent true -> true must not toggle.
  EXPECT_FALSE(state.Advance(true));
  EXPECT_FALSE(state.Advance(true));
  // true -> false stays unchanged; false -> true toggles.
  EXPECT_FALSE(state.Advance(false));
  EXPECT_TRUE(state.Advance(true));
  // Idempotent true -> true stays true.
  EXPECT_TRUE(state.Advance(true));
  // Second manual -> auto flips back to false.
  EXPECT_TRUE(state.Advance(false));
  EXPECT_FALSE(state.Advance(true));
}

TEST(ZedParametersTest, StereoVideoParametersAutoAndManual) {
  CameraSettings settings;
  settings.auto_exposure = true;
  settings.auto_white_balance = true;
  settings.denoising = 70;

  auto untrimmed = ToMap(
      BuildWrapperVideoParameters(settings, /*stereo=*/true, /*trim=*/false));
  EXPECT_EQ(untrimmed.count("video.auto_exposure_gain"), 0u);
  EXPECT_EQ(untrimmed["video.auto_exposure_time_range_min"].get<int64_t>(),
            kExposureMinUs);
  EXPECT_EQ(untrimmed["video.auto_exposure_time_range_max"].get<int64_t>(),
            kExposureMaxUs);
  EXPECT_EQ(untrimmed["video.auto_analog_gain_range_min"].get<int64_t>(),
            kGainMinMdb);
  EXPECT_EQ(untrimmed["video.auto_analog_gain_range_max"].get<int64_t>(),
            kAutoGainMaxMdb);
  EXPECT_EQ(untrimmed["video.auto_digital_gain_range_min"].get<int64_t>(),
            kManualDigitalGain);
  EXPECT_EQ(untrimmed["video.auto_digital_gain_range_max"].get<int64_t>(),
            kManualDigitalGain);
  EXPECT_TRUE(untrimmed["video.auto_whitebalance"].get<bool>());
  EXPECT_EQ(untrimmed.count("video.whitebalance_temperature"), 0u);
  EXPECT_EQ(untrimmed["video.denoising"].get<int64_t>(), 70);

  auto trimmed = ToMap(
      BuildWrapperVideoParameters(settings, /*stereo=*/true, /*trim=*/true));
  EXPECT_EQ(trimmed["video.auto_exposure_time_range_max"].get<int64_t>(),
            kExposureMaxUs - 1);
  EXPECT_EQ(trimmed["video.auto_analog_gain_range_max"].get<int64_t>(),
            kAutoGainMaxMdb - 1);

  settings.auto_exposure = false;
  settings.exposure_us = 12345;
  settings.gain_mdb = 4500;
  settings.auto_white_balance = false;
  settings.white_balance_100k = 52;
  auto manual = ToMap(
      BuildWrapperVideoParameters(settings, /*stereo=*/true, /*trim=*/true));
  EXPECT_EQ(manual["video.auto_exposure_time_range_min"].get<int64_t>(), 12345);
  EXPECT_EQ(manual["video.auto_exposure_time_range_max"].get<int64_t>(), 12345);
  EXPECT_EQ(manual["video.auto_analog_gain_range_min"].get<int64_t>(), 4500);
  EXPECT_EQ(manual["video.auto_analog_gain_range_max"].get<int64_t>(), 4500);
  EXPECT_FALSE(manual["video.auto_whitebalance"].get<bool>());
  EXPECT_EQ(manual["video.whitebalance_temperature"].get<int64_t>(), 52);
}

TEST(ZedParametersTest,
     MonoVideoParametersAutoHasNoSentinelsAndManualSetsScalars) {
  CameraSettings settings;
  settings.auto_exposure = true;
  settings.auto_white_balance = true;
  settings.denoising = 65;

  auto auto_map =
      ToMap(BuildWrapperVideoParameters(settings, /*stereo=*/false));
  EXPECT_TRUE(auto_map["video.auto_exposure"].get<bool>());
  EXPECT_EQ(auto_map.count("video.exposure_time"), 0u);
  EXPECT_TRUE(auto_map["video.auto_analog_gain"].get<bool>());
  EXPECT_EQ(auto_map.count("video.analog_gain"), 0u);
  EXPECT_EQ(auto_map["video.auto_analog_gain_range_max"].get<int64_t>(),
            kAutoGainMaxMdb);
  EXPECT_FALSE(auto_map["video.auto_digital_gain"].get<bool>());
  EXPECT_EQ(auto_map["video.digital_gain"].get<int64_t>(), kManualDigitalGain);
  EXPECT_TRUE(auto_map["video.auto_whitebalance"].get<bool>());
  EXPECT_EQ(auto_map.count("video.whitebalance_temperature"), 0u);
  EXPECT_EQ(auto_map["video.denoising"].get<int64_t>(), 65);

  settings.auto_exposure = false;
  settings.exposure_us = 8000;
  settings.gain_mdb = 3000;
  settings.auto_white_balance = false;
  settings.white_balance_100k = 48;
  auto manual_map =
      ToMap(BuildWrapperVideoParameters(settings, /*stereo=*/false));
  EXPECT_FALSE(manual_map["video.auto_exposure"].get<bool>());
  EXPECT_EQ(manual_map["video.exposure_time"].get<int64_t>(), 8000);
  EXPECT_FALSE(manual_map["video.auto_analog_gain"].get<bool>());
  EXPECT_EQ(manual_map["video.analog_gain"].get<int64_t>(), 3000);
  EXPECT_FALSE(manual_map["video.auto_whitebalance"].get<bool>());
  EXPECT_EQ(manual_map["video.whitebalance_temperature"].get<int64_t>(), 48);
}

TEST(ZedParametersTest, DiffWrapperParametersFiltersUnchangedValues) {
  const std::vector<rclcpp::Parameter> desired = {
      rclcpp::Parameter("video.gamma", 8),
      rclcpp::Parameter("video.saturation", 6),
      rclcpp::Parameter("video.denoising", 50),
  };
  const std::map<std::string, rclcpp::ParameterValue> current = {
      {"video.gamma", rclcpp::ParameterValue(8)},
      {"video.saturation", rclcpp::ParameterValue(4)},
      {"video.denoising", rclcpp::ParameterValue(50)},
  };
  const std::vector<rclcpp::Parameter> changes = DiffWrapperParameters(
      desired,
      [&current](const std::string& name) { return current.at(name); });
  ASSERT_EQ(changes.size(), 1u);
  EXPECT_EQ(changes[0].get_name(), "video.saturation");
  EXPECT_EQ(changes[0].as_int(), 6);
}

TEST(ZedParametersTest, BuildWrapperOptionsSetsStereoAndMonoOverrides) {
  DiscoveredCamera stereo_cam{
      .serial = "92232809",
      .serial_number = 92232809,
      .driver_type = "zed_x_nano",
      .wrapper_model = "zedxnano",
      .stereo = true,
  };
  CameraSettings settings;
  settings.fps = 60;
  settings.denoising = 33;

  const rclcpp::NodeOptions stereo_opts =
      BuildWrapperOptions(stereo_cam, settings);
  EXPECT_TRUE(stereo_opts.use_intra_process_comms());
  auto stereo_overrides = ToMap(stereo_opts.parameter_overrides());
  EXPECT_EQ(stereo_overrides["general.camera_model"].get<std::string>(),
            "zedxnano");
  EXPECT_EQ(stereo_overrides["general.serial_number"].get<int64_t>(), 92232809);
  EXPECT_EQ(stereo_overrides[std::string(kWrapperFrameRate)].get<int64_t>(),
            60);
  EXPECT_FALSE(stereo_overrides["general.self_calib"].get<bool>());
  EXPECT_DOUBLE_EQ(stereo_overrides["general.pub_frame_rate"].get<double>(),
                   0.0);
  EXPECT_TRUE(stereo_overrides["video.publish_raw"].get<bool>());
  EXPECT_TRUE(stereo_overrides["video.enable_24bit_output"].get<bool>());
  EXPECT_EQ(stereo_overrides["video.denoising"].get<int64_t>(), 33);

  DiscoveredCamera mono_cam{
      .serial = "301985430",
      .serial_number = 301985430,
      .driver_type = "zed_x_one_gs",
      .wrapper_model = "zedxonegs",
      .stereo = false,
  };
  const rclcpp::NodeOptions mono_opts = BuildWrapperOptions(mono_cam, settings);
  auto mono_overrides = ToMap(mono_opts.parameter_overrides());
  EXPECT_EQ(mono_overrides["general.camera_model"].get<std::string>(),
            "zedxonegs");
  EXPECT_EQ(mono_overrides.count("general.self_calib"), 0u);
  EXPECT_EQ(mono_overrides["video.denoising"].get<int64_t>(), 33);
}

}  // namespace
}  // namespace flowstate_zed
