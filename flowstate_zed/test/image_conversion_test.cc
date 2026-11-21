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

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "absl/status/status.h"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

namespace flowstate_zed {
namespace {

std_msgs::msg::Header TestHeader() {
  std_msgs::msg::Header h;
  h.frame_id = "zed_left_optical";
  h.stamp.sec = 42;
  h.stamp.nanosec = 123456;
  return h;
}

TEST(ImageConversionTest, ConvertsPackedBgr8ToRgb8) {
  const std::vector<uint8_t> src = {
      10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120,
  };
  sensor_msgs::msg::Image dst;
  const absl::Status status =
      ConvertBgrToRgb8(src.data(), /*width=*/2, /*height=*/2, /*src_step=*/6,
                       src.size(), /*channels=*/3, TestHeader(), dst);
  ASSERT_TRUE(status.ok()) << status;
  EXPECT_EQ(dst.header.frame_id, "zed_left_optical");
  EXPECT_EQ(dst.header.stamp.sec, 42);
  EXPECT_EQ(dst.header.stamp.nanosec, 123456u);
  EXPECT_EQ(dst.width, 2u);
  EXPECT_EQ(dst.height, 2u);
  EXPECT_EQ(dst.step, 6u);
  EXPECT_EQ(dst.encoding, sensor_msgs::image_encodings::RGB8);
  EXPECT_EQ(dst.is_bigendian, 0u);
  const std::vector<uint8_t> expected = {
      30, 20, 10, 60, 50, 40, 90, 80, 70, 120, 110, 100,
  };
  EXPECT_EQ(dst.data, expected);
}

TEST(ImageConversionTest, RepacksPaddedBgr8ToTightRgb8) {
  // 2x2 BGR8 with 2 bytes of row padding (src_step = 8).
  const std::vector<uint8_t> src = {
      1, 2, 3, 4, 5, 6, 0xAA, 0xBB, 7, 8, 9, 10, 11, 12, 0xCC, 0xDD,
  };
  sensor_msgs::msg::Image dst;
  const absl::Status status =
      ConvertBgrToRgb8(src.data(), /*width=*/2, /*height=*/2, /*src_step=*/8,
                       src.size(), /*channels=*/3, TestHeader(), dst);
  ASSERT_TRUE(status.ok()) << status;
  EXPECT_EQ(dst.step, 6u);
  const std::vector<uint8_t> expected = {
      3, 2, 1, 6, 5, 4, 9, 8, 7, 12, 11, 10,
  };
  EXPECT_EQ(dst.data, expected);
}

TEST(ImageConversionTest, ConvertsBgra8WithPaddingToTightRgb8) {
  // 2x1 BGRA8 with 4 bytes of trailing padding (src_step = 12).
  const std::vector<uint8_t> src = {
      11, 22, 33, 255, 44, 55, 66, 128, 0, 0, 0, 0,
  };
  sensor_msgs::msg::Image dst;
  const absl::Status status =
      ConvertBgrToRgb8(src.data(), /*width=*/2, /*height=*/1, /*src_step=*/12,
                       src.size(), /*channels=*/4, TestHeader(), dst);
  ASSERT_TRUE(status.ok()) << status;
  EXPECT_EQ(dst.step, 6u);
  EXPECT_EQ(dst.data, (std::vector<uint8_t>{33, 22, 11, 66, 55, 44}));
}

TEST(ImageConversionTest, HandlesEmptyZeroByZeroImage) {
  sensor_msgs::msg::Image dst;
  const absl::Status status =
      ConvertBgrToRgb8(nullptr, /*width=*/0, /*height=*/0, /*src_step=*/0,
                       /*src_bytes=*/0, /*channels=*/3, TestHeader(), dst);
  ASSERT_TRUE(status.ok()) << status;
  EXPECT_EQ(dst.width, 0u);
  EXPECT_EQ(dst.height, 0u);
  EXPECT_TRUE(dst.data.empty());
}

TEST(ImageConversionTest, RejectsInvalidInputs) {
  const std::vector<uint8_t> buf(16, 0);
  sensor_msgs::msg::Image dst;
  EXPECT_EQ(
      ConvertBgrToRgb8(buf.data(), 2, 2, 6, buf.size(), 1, TestHeader(), dst)
          .code(),
      absl::StatusCode::kInvalidArgument);
  EXPECT_EQ(
      ConvertBgrToRgb8(buf.data(), 2, 2, 4, buf.size(), 3, TestHeader(), dst)
          .code(),
      absl::StatusCode::kInvalidArgument);
  EXPECT_EQ(
      ConvertBgrToRgb8(buf.data(), 2, 2, 6, 10, 3, TestHeader(), dst).code(),
      absl::StatusCode::kInvalidArgument);
  EXPECT_EQ(ConvertBgrToRgb8(nullptr, 2, 2, 6, 12, 3, TestHeader(), dst).code(),
            absl::StatusCode::kInvalidArgument);
}

}  // namespace
}  // namespace flowstate_zed
