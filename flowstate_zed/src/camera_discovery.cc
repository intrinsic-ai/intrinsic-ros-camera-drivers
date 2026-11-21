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

#include <algorithm>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"

namespace flowstate_zed {

std::optional<DiscoveredCamera> ClassifyDevice(const RawDevice& device) {
  DiscoveredCamera camera;
  camera.serial = std::to_string(device.serial_number);
  camera.serial_number = device.serial_number;
  switch (device.model) {
    case CameraModel::kZedX:
      camera.driver_type = "zed_x";
      camera.wrapper_model = "zedx";
      return camera;
    case CameraModel::kZedXMini:
      camera.driver_type = "zed_x_mini";
      camera.wrapper_model = "zedxm";
      return camera;
    case CameraModel::kZedXNano:
      camera.driver_type = "zed_x_nano";
      camera.wrapper_model = "zedxnano";
      return camera;
    case CameraModel::kZedXOneGs:
      camera.driver_type = "zed_x_one_gs";
      camera.wrapper_model = "zedxonegs";
      camera.stereo = false;
      return camera;
    case CameraModel::kUnsupported:
      return std::nullopt;
  }
  return std::nullopt;
}

DiscoveryPass CollectSupportedDevices(
    absl::Span<const RawDevice> devices,
    const std::function<void(const RawDevice&)>& on_unsupported) {
  DiscoveryPass pass;
  for (const RawDevice& device : devices) {
    std::optional<DiscoveredCamera> camera = ClassifyDevice(device);
    if (!camera) {
      if (on_unsupported) {
        on_unsupported(device);
      }
      continue;
    }
    if (!device.available) {
      pass.any_unavailable = true;
    }
    const bool known =
        std::ranges::any_of(pass.cameras, [&camera](const DiscoveredCamera& c) {
          return c.serial_number == camera->serial_number;
        });
    if (!known) {
      pass.cameras.push_back(std::move(*camera));
    }
  }
  return pass;
}

std::vector<DiscoveredCamera> SettleDevices(
    const std::function<DiscoveryPass(int attempt)>& enumerate_pass,
    const std::function<void()>& reboot_gmsl,
    const std::function<bool(std::chrono::milliseconds)>& sleep_while_ok,
    const std::function<bool()>& is_ok, const SettleConfig& config) {
  std::vector<DiscoveredCamera> best_partial;
  std::optional<size_t> stable_ready_count;
  bool reset_attempted = false;

  for (int attempt = 0; attempt < config.max_attempts && is_ok(); ++attempt) {
    if (attempt > 0) {
      const std::chrono::milliseconds delay = stable_ready_count.has_value()
                                                  ? config.settle_delay
                                                  : config.retry_delay;
      if (!sleep_while_ok(delay)) return {};
    }

    DiscoveryPass pass = enumerate_pass(attempt);
    if (!is_ok()) return {};

    if (pass.cameras.size() >= best_partial.size() && !pass.cameras.empty()) {
      best_partial = pass.cameras;
    }

    if (!pass.cameras.empty() && !pass.any_unavailable) {
      if (stable_ready_count == pass.cameras.size()) {
        return pass.cameras;
      }
      stable_ready_count = pass.cameras.size();
      continue;
    }

    stable_ready_count.reset();
    if (!reset_attempted && is_ok()) {
      reset_attempted = true;
      reboot_gmsl();
    }
  }
  if (!is_ok()) return {};
  return best_partial;
}

std::string ColorImageTopic(absl::string_view serial, bool stereo) {
  return absl::StrCat("/zed/camera_", serial, "/", kWrapperNodeName,
                      stereo ? "/left" : "/rgb", "/color/raw/image");
}

std::string RightImageTopic(absl::string_view serial) {
  return absl::StrCat("/zed/camera_", serial, "/", kWrapperNodeName,
                      "/right/color/raw/image");
}

std::string InfoTopic(absl::string_view image_topic) {
  // ".../raw/image" -> ".../raw/camera_info".
  return absl::StrCat(image_topic.substr(0, image_topic.rfind('/')),
                      "/camera_info");
}

}  // namespace flowstate_zed
