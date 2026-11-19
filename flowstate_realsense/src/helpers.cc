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

#include "flowstate_realsense/helpers.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/ascii.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace flowstate_realsense {
namespace {

// The driver publishes depth as 16UC1 millimeters whatever the device's own
// depth unit (its fix_depth_scale); Flowstate expects 32FC1 meters.
constexpr float kMillimeterToMeter = 0.001f;

}  // namespace

absl::StatusOr<sensor_msgs::msg::Image> ConvertDepthToMeters(
    const sensor_msgs::msg::Image& depth_mm) {
  if (depth_mm.encoding != sensor_msgs::image_encodings::TYPE_16UC1) {
    return absl::InvalidArgumentError(absl::StrCat(
        "depth encoding ", depth_mm.encoding, ", expected 16UC1 millimeters"));
  }
  if (depth_mm.is_bigendian != 0) {
    return absl::InvalidArgumentError(
        "big-endian 16UC1 depth images are not supported");
  }

  const size_t width = depth_mm.width;
  const size_t height = depth_mm.height;
  const size_t in_step = depth_mm.step;
  const size_t out_step = width * sizeof(float);
  if (out_step > std::numeric_limits<uint32_t>::max()) {
    return absl::InvalidArgumentError(
        absl::StrCat("depth width too large for 32FC1 step: ", width));
  }
  if (height > 0 && out_step > std::numeric_limits<size_t>::max() / height) {
    return absl::InvalidArgumentError(absl::StrCat(
        "depth dimensions overflow output size: ", width, "x", height));
  }
  if (in_step < width * sizeof(uint16_t)) {
    return absl::InvalidArgumentError(absl::StrCat(
        "depth row step too small: ", in_step, " bytes for width ", width));
  }
  if (height > 0 && in_step > depth_mm.data.size() / height) {
    return absl::InvalidArgumentError(absl::StrCat(
        "depth buffer too small: ", depth_mm.data.size(), " bytes for ", width,
        "x", height, " with step ", in_step));
  }

  sensor_msgs::msg::Image depth;
  depth.header = depth_mm.header;
  depth.height = depth_mm.height;
  depth.width = depth_mm.width;
  depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  depth.is_bigendian = 0;
  depth.step = static_cast<uint32_t>(out_step);
  depth.data.resize(out_step * height);
  for (size_t row = 0; row < height; ++row) {
    const uint8_t* src = depth_mm.data.data() + row * in_step;
    uint8_t* dst = depth.data.data() + row * out_step;
    for (size_t col = 0; col < width; ++col) {
      uint16_t mm = 0;
      std::memcpy(&mm, src + col * sizeof(uint16_t), sizeof(mm));
      const float meters = static_cast<float>(mm) * kMillimeterToMeter;
      std::memcpy(dst + col * sizeof(float), &meters, sizeof(meters));
    }
  }
  return depth;
}

std::optional<ImageDimensions> ParseProfileDimensions(
    absl::string_view profile) {
  const std::vector<absl::string_view> parts =
      absl::StrSplit(profile, absl::ByAnyChar("xX,"));
  if (parts.size() != 3) return std::nullopt;
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t fps = 0;
  if (!absl::SimpleAtoi(absl::StripAsciiWhitespace(parts[0]), &width) ||
      !absl::SimpleAtoi(absl::StripAsciiWhitespace(parts[1]), &height) ||
      !absl::SimpleAtoi(absl::StripAsciiWhitespace(parts[2]), &fps)) {
    return std::nullopt;
  }
  if (width == 0 || height == 0 || fps == 0) return std::nullopt;
  return ImageDimensions{.width = width, .height = height};
}

absl::StatusOr<std::string> ParseComponentLibrary(
    absl::string_view resource_content, absl::string_view prefix,
    absl::string_view class_name) {
  for (absl::string_view line : absl::StrSplit(resource_content, '\n')) {
    const std::vector<absl::string_view> parts = absl::StrSplit(line, ';');
    if (parts.size() != 2 || parts[0] != class_name) continue;
    const absl::string_view path = parts[1];
    if (path.empty()) continue;
    return path.front() == '/' ? std::string(path)
                               : absl::StrCat(prefix, "/", path);
  }
  return absl::NotFoundError(
      absl::StrCat("component ", class_name, " not found in resource index"));
}

}  // namespace flowstate_realsense
