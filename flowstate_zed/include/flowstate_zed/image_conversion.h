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

#ifndef FLOWSTATE_ZED_IMAGE_CONVERSION_H_
#define FLOWSTATE_ZED_IMAGE_CONVERSION_H_

#include <cstddef>
#include <cstdint>

#include "absl/status/status.h"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

namespace flowstate_zed {

// Copies and converts raw BGR8 (channels == 3) or BGRA8 (channels == 4) pixels
// into `dst` as a tightly packed RGB8 image with `header`.
absl::Status ConvertBgrToRgb8(const uint8_t* src, uint32_t width,
                              uint32_t height, uint32_t src_step,
                              size_t src_bytes, size_t channels,
                              const std_msgs::msg::Header& header,
                              sensor_msgs::msg::Image& dst);

}  // namespace flowstate_zed

#endif  // FLOWSTATE_ZED_IMAGE_CONVERSION_H_
