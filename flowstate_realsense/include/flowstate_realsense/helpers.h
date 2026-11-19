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

#ifndef FLOWSTATE_REALSENSE_HELPERS_H_
#define FLOWSTATE_REALSENSE_HELPERS_H_

#include <cstdint>
#include <optional>
#include <string>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "sensor_msgs/msg/image.hpp"

namespace flowstate_realsense {

struct ImageDimensions {
  uint32_t width = 0;
  uint32_t height = 0;
};

// Converts a little-endian 16UC1 depth image in millimeters to a tightly packed
// little-endian 32FC1 depth image in meters (0 stays 0). Rejects unsupported
// encodings, big-endian images, overflowing dimensions, undersized row steps,
// or truncated buffers.
absl::StatusOr<sensor_msgs::msg::Image> ConvertDepthToMeters(
    const sensor_msgs::msg::Image& depth_mm);

// Parses a realsense2_camera video profile string ("<w>x<h>x<fps>", accepting
// 'x', 'X', or ',' separators and surrounding whitespace) and returns its
// positive pixel dimensions, or std::nullopt if malformed or non-positive.
std::optional<ImageDimensions> ParseProfileDimensions(
    absl::string_view profile);

// Parses the ament `rclcpp_components` resource file content and returns the
// resolved shared library path for `class_name`, joining relative paths with
// `prefix`. Returns NotFoundError if `class_name` is absent.
absl::StatusOr<std::string> ParseComponentLibrary(
    absl::string_view resource_content, absl::string_view prefix,
    absl::string_view class_name);

}  // namespace flowstate_realsense

#endif  // FLOWSTATE_REALSENSE_HELPERS_H_
