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

#ifndef FLOWSTATE_ZED_CAMERA_DISCOVERY_H_
#define FLOWSTATE_ZED_CAMERA_DISCOVERY_H_

#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "absl/strings/string_view.h"
#include "absl/types/span.h"

namespace flowstate_zed {

inline constexpr absl::string_view kWrapperNodeName = "zed_node";

enum class CameraModel {
  kZedX,
  kZedXMini,
  kZedXNano,
  kZedXOneGs,
  kUnsupported,
};

// SDK-independent view of one enumerated device.
struct RawDevice {
  uint32_t serial_number = 0;
  CameraModel model = CameraModel::kUnsupported;
  bool available = false;
  std::string model_label;
};

// One supported ZED X camera ready to be managed by an AdapterNode.
struct DiscoveredCamera {
  std::string serial;          // Decimal serial number.
  uint32_t serial_number = 0;  // Numeric serial number.
  std::string driver_type;     // Flowstate driver_type and node prefix.
  std::string wrapper_model;   // Wrapper `general.camera_model` value.
  bool stereo = true;          // False for ZED X One models.
};

struct DiscoveryPass {
  std::vector<DiscoveredCamera> cameras;
  bool any_unavailable = false;
};

struct SettleConfig {
  int max_attempts = 5;
  // Wait before confirming a pass where all discovered cameras were AVAILABLE.
  std::chrono::milliseconds settle_delay{1000};
  // Wait before retrying after an empty pass or a GMSL/Argus reset.
  std::chrono::milliseconds retry_delay{2000};
};

// Maps a supported ZED X device to a DiscoveredCamera, or nullopt if
// unsupported.
std::optional<DiscoveredCamera> ClassifyDevice(const RawDevice& device);

// Filters `devices` to supported models, deduplicates by `serial_number`, and
// records whether any supported device is unavailable. Calls `on_unsupported`
// (when non-null) for each skipped device.
DiscoveryPass CollectSupportedDevices(
    absl::Span<const RawDevice> devices,
    const std::function<void(const RawDevice&)>& on_unsupported = nullptr);

// Runs up to `config.max_attempts` discovery passes until two consecutive
// non-empty passes report the same count of all-AVAILABLE cameras. Triggers
// `reboot_gmsl` at most once if a pass is empty or contains an unavailable
// camera. Returns {} if `is_ok()` or `sleep_while_ok()` turns false.
std::vector<DiscoveredCamera> SettleDevices(
    const std::function<DiscoveryPass(int attempt)>& enumerate_pass,
    const std::function<void()>& reboot_gmsl,
    const std::function<bool(std::chrono::milliseconds)>& sleep_while_ok,
    const std::function<bool()>& is_ok, const SettleConfig& config = {});

std::string ColorImageTopic(absl::string_view serial, bool stereo);
std::string RightImageTopic(absl::string_view serial);
std::string InfoTopic(absl::string_view image_topic);

}  // namespace flowstate_zed

#endif  // FLOWSTATE_ZED_CAMERA_DISCOVERY_H_
