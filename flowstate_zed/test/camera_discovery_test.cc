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

#include "flowstate_zed/camera_discovery.h"

#include <gtest/gtest.h>

#include <chrono>
#include <optional>
#include <vector>

namespace flowstate_zed {
namespace {

TEST(CameraDiscoveryTest, ClassifiesAllSupportedModelsAndRejectsUnsupported) {
  auto zedx = ClassifyDevice({100, CameraModel::kZedX, true, "ZED X"});
  ASSERT_TRUE(zedx.has_value());
  EXPECT_EQ(zedx->serial, "100");
  EXPECT_EQ(zedx->driver_type, "zed_x");
  EXPECT_EQ(zedx->wrapper_model, "zedx");
  EXPECT_TRUE(zedx->stereo);

  auto zedxm =
      ClassifyDevice({101, CameraModel::kZedXMini, true, "ZED X Mini"});
  ASSERT_TRUE(zedxm.has_value());
  EXPECT_EQ(zedxm->driver_type, "zed_x_mini");
  EXPECT_EQ(zedxm->wrapper_model, "zedxm");
  EXPECT_TRUE(zedxm->stereo);

  auto nano = ClassifyDevice({102, CameraModel::kZedXNano, true, "ZED X Nano"});
  ASSERT_TRUE(nano.has_value());
  EXPECT_EQ(nano->driver_type, "zed_x_nano");
  EXPECT_EQ(nano->wrapper_model, "zedxnano");
  EXPECT_TRUE(nano->stereo);

  auto one =
      ClassifyDevice({103, CameraModel::kZedXOneGs, true, "ZED XOne GS"});
  ASSERT_TRUE(one.has_value());
  EXPECT_EQ(one->driver_type, "zed_x_one_gs");
  EXPECT_EQ(one->wrapper_model, "zedxonegs");
  EXPECT_FALSE(one->stereo);

  EXPECT_FALSE(ClassifyDevice({104, CameraModel::kUnsupported, true, "ZED 2i"})
                   .has_value());
}

TEST(CameraDiscoveryTest, CollectSupportedDeduplicatesAndTracksAvailability) {
  std::vector<uint32_t> unsupported_serials;
  const std::vector<RawDevice> raw = {
      {10, CameraModel::kZedXNano, true, "ZED X Nano"},
      {10, CameraModel::kZedXNano, true, "ZED X Nano"},
      {20, CameraModel::kUnsupported, false, "ZED 2i"},
      {30, CameraModel::kZedXOneGs, true, "ZED XOne GS"},
  };
  const DiscoveryPass pass =
      CollectSupportedDevices(raw, [&unsupported_serials](const RawDevice& d) {
        unsupported_serials.push_back(d.serial_number);
      });
  ASSERT_EQ(pass.cameras.size(), 2u);
  EXPECT_EQ(pass.cameras[0].serial_number, 10u);
  EXPECT_EQ(pass.cameras[1].serial_number, 30u);
  EXPECT_FALSE(pass.any_unavailable);
  EXPECT_EQ(unsupported_serials, (std::vector<uint32_t>{20u}));
}

TEST(CameraDiscoveryTest, SettleDevicesConfirmsTwoStablePasses) {
  const DiscoveredCamera cam =
      *ClassifyDevice({1, CameraModel::kZedXNano, true});
  int passes = 0;
  int reboots = 0;
  std::vector<std::chrono::milliseconds> sleeps;
  const SettleConfig cfg{.max_attempts = 5,
                         .settle_delay = std::chrono::milliseconds(1000),
                         .retry_delay = std::chrono::milliseconds(2000)};

  const auto result = SettleDevices(
      [&](int) {
        ++passes;
        return DiscoveryPass{{cam}, false};
      },
      [&]() { ++reboots; },
      [&](std::chrono::milliseconds d) {
        sleeps.push_back(d);
        return true;
      },
      []() { return true; }, cfg);

  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].serial_number, 1u);
  EXPECT_EQ(passes, 2);
  EXPECT_EQ(reboots, 0);
  EXPECT_EQ(sleeps, (std::vector<std::chrono::milliseconds>{cfg.settle_delay}));
}

TEST(CameraDiscoveryTest, SettleDevicesDropsDeadCameraAfterReboot) {
  // Regression test: pass 0 sees 2 cameras (1 unavailable), triggers one-shot
  // reboot, then passes 1 & 2 settle on 1 available camera. Must return only
  // the 1 settled camera, not the stale 2-camera list from pass 0.
  const DiscoveredCamera cam1 =
      *ClassifyDevice({1, CameraModel::kZedXNano, true});
  const DiscoveredCamera cam2 =
      *ClassifyDevice({2, CameraModel::kZedXOneGs, false});
  int reboots = 0;
  std::vector<std::chrono::milliseconds> sleeps;
  const SettleConfig cfg{.max_attempts = 5,
                         .settle_delay = std::chrono::milliseconds(1000),
                         .retry_delay = std::chrono::milliseconds(2000)};

  const auto result = SettleDevices(
      [&](int attempt) {
        if (attempt == 0) {
          return DiscoveryPass{{cam1, cam2}, /*any_unavailable=*/true};
        }
        return DiscoveryPass{{cam1}, /*any_unavailable=*/false};
      },
      [&]() { ++reboots; },
      [&](std::chrono::milliseconds d) {
        sleeps.push_back(d);
        return true;
      },
      []() { return true; }, cfg);

  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].serial_number, 1u);
  EXPECT_EQ(reboots, 1);
  EXPECT_EQ(sleeps, (std::vector<std::chrono::milliseconds>{cfg.retry_delay,
                                                            cfg.settle_delay}));
}

TEST(CameraDiscoveryTest, SettleDevicesStopsWhenNotOk) {
  bool ok = true;
  const auto result = SettleDevices(
      [&](int) {
        ok = false;
        return DiscoveryPass{};
      },
      []() {}, [](std::chrono::milliseconds) { return true; },
      [&]() { return ok; });
  EXPECT_TRUE(result.empty());
}

TEST(CameraDiscoveryTest, TopicHelpers) {
  EXPECT_EQ(ColorImageTopic("92232809", /*stereo=*/true),
            "/zed/camera_92232809/zed_node/left/color/raw/image");
  EXPECT_EQ(ColorImageTopic("301985430", /*stereo=*/false),
            "/zed/camera_301985430/zed_node/rgb/color/raw/image");
  EXPECT_EQ(RightImageTopic("92232809"),
            "/zed/camera_92232809/zed_node/right/color/raw/image");
  EXPECT_EQ(InfoTopic("/zed/camera_92232809/zed_node/left/color/raw/image"),
            "/zed/camera_92232809/zed_node/left/color/raw/camera_info");
}

}  // namespace
}  // namespace flowstate_zed
