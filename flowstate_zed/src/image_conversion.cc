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

#include "flowstate_zed/image_conversion.h"

#include <cstddef>
#include <cstdint>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "sensor_msgs/image_encodings.hpp"

namespace flowstate_zed {

absl::Status ConvertBgrToRgb8(const uint8_t* src, uint32_t width,
                              uint32_t height, uint32_t src_step,
                              size_t src_bytes, size_t channels,
                              const std_msgs::msg::Header& header,
                              sensor_msgs::msg::Image& dst) {
  if (channels != 3 && channels != 4) {
    return absl::InvalidArgumentError(
        absl::StrCat("Unsupported channel count: ", channels));
  }
  const size_t w = width;
  const size_t h = height;
  const size_t in_step = src_step;
  const size_t min_step = w * channels;
  if (in_step < min_step) {
    return absl::InvalidArgumentError(
        absl::StrCat("Row step too small: ", in_step, " bytes for width ", w,
                     " with ", channels, " channels"));
  }
  const size_t required_bytes = in_step * h;
  if (src_bytes < required_bytes) {
    return absl::InvalidArgumentError(
        absl::StrCat("Image buffer too small: ", src_bytes, " bytes for ", w,
                     "x", h, " with step ", in_step));
  }
  if (required_bytes > 0 && src == nullptr) {
    return absl::InvalidArgumentError("Image buffer pointer is null");
  }

  const size_t out_step = 3 * w;
  dst.header = header;
  dst.width = width;
  dst.height = height;
  dst.encoding = sensor_msgs::image_encodings::RGB8;
  dst.is_bigendian = false;
  dst.step = static_cast<uint32_t>(out_step);
  dst.data.resize(out_step * h);

  uint8_t* out = dst.data.data();
  for (size_t y = 0; y < h; ++y) {
    const uint8_t* row_src = src + y * in_step;
    uint8_t* row_dst = out + y * out_step;
    for (size_t x = 0; x < w; ++x) {
      row_dst[0] = row_src[2];
      row_dst[1] = row_src[1];
      row_dst[2] = row_src[0];
      row_src += channels;
      row_dst += 3;
    }
  }
  return absl::OkStatus();
}

}  // namespace flowstate_zed
