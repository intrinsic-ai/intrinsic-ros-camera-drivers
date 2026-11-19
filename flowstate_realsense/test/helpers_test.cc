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
#include "absl/strings/match.h"
#include "gtest/gtest.h"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace flowstate_realsense {
namespace {

float ReadFloat(const sensor_msgs::msg::Image& image, uint32_t row,
                uint32_t col) {
  float value = 0.0f;
  const size_t offset =
      static_cast<size_t>(row) * image.step + col * sizeof(float);
  std::memcpy(&value, image.data.data() + offset, sizeof(value));
  return value;
}

void WriteUint16(std::vector<uint8_t>& buffer, size_t offset, uint16_t value) {
  std::memcpy(buffer.data() + offset, &value, sizeof(value));
}

TEST(ConvertDepthToMetersTest, ConvertsValidPaddedImage) {
  sensor_msgs::msg::Image depth_mm;
  depth_mm.header.frame_id = "camera_depth_optical_frame";
  depth_mm.width = 2;
  depth_mm.height = 2;
  depth_mm.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  depth_mm.is_bigendian = 0;
  depth_mm.step = 6;  // 4 payload bytes + 2 padding bytes per row.
  depth_mm.data.assign(12, 0xAA);
  WriteUint16(depth_mm.data, 0, 0);
  WriteUint16(depth_mm.data, 2, 1250);
  WriteUint16(depth_mm.data, 6, 500);
  WriteUint16(depth_mm.data, 8, 65535);

  const absl::StatusOr<sensor_msgs::msg::Image> depth_m =
      ConvertDepthToMeters(depth_mm);
  ASSERT_TRUE(depth_m.ok()) << depth_m.status();
  EXPECT_EQ(depth_m->header.frame_id, "camera_depth_optical_frame");
  EXPECT_EQ(depth_m->width, 2u);
  EXPECT_EQ(depth_m->height, 2u);
  EXPECT_EQ(depth_m->encoding, sensor_msgs::image_encodings::TYPE_32FC1);
  EXPECT_EQ(depth_m->is_bigendian, 0u);
  EXPECT_EQ(depth_m->step, 8u);
  ASSERT_EQ(depth_m->data.size(), 16u);
  EXPECT_FLOAT_EQ(ReadFloat(*depth_m, 0, 0), 0.0f);
  EXPECT_FLOAT_EQ(ReadFloat(*depth_m, 0, 1), 1.25f);
  EXPECT_FLOAT_EQ(ReadFloat(*depth_m, 1, 0), 0.5f);
  EXPECT_FLOAT_EQ(ReadFloat(*depth_m, 1, 1), 65.535f);
}

TEST(ConvertDepthToMetersTest, ConvertsEmptyImage) {
  sensor_msgs::msg::Image depth_mm;
  depth_mm.width = 0;
  depth_mm.height = 0;
  depth_mm.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  depth_mm.step = 0;

  const absl::StatusOr<sensor_msgs::msg::Image> depth_m =
      ConvertDepthToMeters(depth_mm);
  ASSERT_TRUE(depth_m.ok()) << depth_m.status();
  EXPECT_EQ(depth_m->step, 0u);
  EXPECT_TRUE(depth_m->data.empty());
}

TEST(ConvertDepthToMetersTest, RejectsWrongEncoding) {
  sensor_msgs::msg::Image depth_mm;
  depth_mm.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  const absl::StatusOr<sensor_msgs::msg::Image> result =
      ConvertDepthToMeters(depth_mm);
  EXPECT_EQ(result.status().code(), absl::StatusCode::kInvalidArgument);
  EXPECT_TRUE(absl::StrContains(result.status().message(), "expected 16UC1"));
}

TEST(ConvertDepthToMetersTest, RejectsBigEndian) {
  sensor_msgs::msg::Image depth_mm;
  depth_mm.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  depth_mm.is_bigendian = 1;
  const absl::StatusOr<sensor_msgs::msg::Image> result =
      ConvertDepthToMeters(depth_mm);
  EXPECT_EQ(result.status().code(), absl::StatusCode::kInvalidArgument);
  EXPECT_TRUE(absl::StrContains(result.status().message(), "big-endian"));
}

TEST(ConvertDepthToMetersTest, RejectsUndersizedStep) {
  sensor_msgs::msg::Image depth_mm;
  depth_mm.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  depth_mm.width = 4;
  depth_mm.height = 1;
  depth_mm.step = 6;  // Needs at least 8 bytes.
  depth_mm.data.resize(8);
  const absl::StatusOr<sensor_msgs::msg::Image> result =
      ConvertDepthToMeters(depth_mm);
  EXPECT_EQ(result.status().code(), absl::StatusCode::kInvalidArgument);
  EXPECT_TRUE(absl::StrContains(result.status().message(), "row step"));
}

TEST(ConvertDepthToMetersTest, RejectsTruncatedDataBuffer) {
  sensor_msgs::msg::Image depth_mm;
  depth_mm.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  depth_mm.width = 2;
  depth_mm.height = 3;
  depth_mm.step = 4;
  depth_mm.data.resize(11);  // Needs 12 bytes.
  const absl::StatusOr<sensor_msgs::msg::Image> result =
      ConvertDepthToMeters(depth_mm);
  EXPECT_EQ(result.status().code(), absl::StatusCode::kInvalidArgument);
  EXPECT_TRUE(absl::StrContains(result.status().message(), "buffer too small"));
}

TEST(ConvertDepthToMetersTest, RejectsOverflowingDimensions) {
  sensor_msgs::msg::Image depth_mm;
  depth_mm.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  depth_mm.width = std::numeric_limits<uint32_t>::max() / 2;
  depth_mm.height = 1;
  depth_mm.step = std::numeric_limits<uint32_t>::max();
  const absl::StatusOr<sensor_msgs::msg::Image> result =
      ConvertDepthToMeters(depth_mm);
  EXPECT_EQ(result.status().code(), absl::StatusCode::kInvalidArgument);
  EXPECT_TRUE(absl::StrContains(result.status().message(), "32FC1 step"));
}

TEST(ParseProfileDimensionsTest, ParsesStandardAndAlternateSeparators) {
  const std::optional<ImageDimensions> standard =
      ParseProfileDimensions("848x480x30");
  ASSERT_TRUE(standard.has_value());
  EXPECT_EQ(standard->width, 848u);
  EXPECT_EQ(standard->height, 480u);

  const std::optional<ImageDimensions> spaced =
      ParseProfileDimensions(" 1280 X 720 , 15 ");
  ASSERT_TRUE(spaced.has_value());
  EXPECT_EQ(spaced->width, 1280u);
  EXPECT_EQ(spaced->height, 720u);
}

TEST(ParseProfileDimensionsTest, RejectsMalformedOrNonPositiveProfiles) {
  EXPECT_FALSE(ParseProfileDimensions("").has_value());
  EXPECT_FALSE(ParseProfileDimensions("1280x720").has_value());
  EXPECT_FALSE(ParseProfileDimensions("1280x720x30x1").has_value());
  EXPECT_FALSE(ParseProfileDimensions("0x0x0").has_value());
  EXPECT_FALSE(ParseProfileDimensions("640x480x0").has_value());
  EXPECT_FALSE(ParseProfileDimensions("-640x480x30").has_value());
  EXPECT_FALSE(ParseProfileDimensions("abcx480x30").has_value());
}

TEST(ParseComponentLibraryTest, ResolvesRelativePathWithPrefix) {
  const absl::StatusOr<std::string> path = ParseComponentLibrary(
      "other::Node;lib/libother.so\n"
      "realsense2_camera::RealSenseNodeFactory;lib/librealsense2_camera.so\n",
      "/opt/ros/jazzy", "realsense2_camera::RealSenseNodeFactory");
  ASSERT_TRUE(path.ok()) << path.status();
  EXPECT_EQ(*path, "/opt/ros/jazzy/lib/librealsense2_camera.so");
}

TEST(ParseComponentLibraryTest, PreservesAbsolutePath) {
  const absl::StatusOr<std::string> path = ParseComponentLibrary(
      "realsense2_camera::RealSenseNodeFactory;/usr/lib/libfoo.so\n",
      "/opt/ros/jazzy", "realsense2_camera::RealSenseNodeFactory");
  ASSERT_TRUE(path.ok()) << path.status();
  EXPECT_EQ(*path, "/usr/lib/libfoo.so");
}

TEST(ParseComponentLibraryTest, ReturnsNotFoundForMissingOrMalformedEntries) {
  const absl::StatusOr<std::string> path = ParseComponentLibrary(
      "\nmalformed_line\na;b;c\nother::Node;lib/libother.so\n",
      "/opt/ros/jazzy", "realsense2_camera::RealSenseNodeFactory");
  EXPECT_EQ(path.status().code(), absl::StatusCode::kNotFound);
}

}  // namespace
}  // namespace flowstate_realsense
