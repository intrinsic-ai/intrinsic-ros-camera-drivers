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

#include "flowstate_zed/frame_cache.h"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "absl/status/status.h"
#include "absl/time/time.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace flowstate_zed {
namespace {

struct FakeFrame {
  int id = 0;
};

rclcpp::Time Stamp(int64_t sec) { return rclcpp::Time(sec, 0, RCL_ROS_TIME); }

std::unique_ptr<sensor_msgs::msg::CameraInfo> MakeInfo(int64_t sec) {
  auto info = std::make_unique<sensor_msgs::msg::CameraInfo>();
  info->header.stamp = Stamp(sec);
  info->width = 1920;
  info->height = 1200;
  return info;
}

TEST(FrameCacheTest, MonoMovesFrameAndPreservesLastStampForNextCall) {
  FrameCache<FakeFrame> cache(/*stereo=*/false);
  cache.OnLeftInfo(MakeInfo(1));
  cache.OnLeftImage(std::make_unique<FakeFrame>(FakeFrame{1}), Stamp(10),
                    Stamp(10));
  EXPECT_TRUE(cache.HasFirstFrame());
  cache.SetWrapperReady(true);

  std::thread producer([&]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    cache.OnLeftImage(std::make_unique<FakeFrame>(FakeFrame{2}), Stamp(20),
                      Stamp(20));
  });
  auto snap1 = cache.WaitAndTakeSnapshot(absl::Seconds(1), absl::Seconds(1));
  producer.join();
  ASSERT_TRUE(snap1.ok()) << snap1.status();
  EXPECT_EQ(snap1->left_frame->id, 2);
  EXPECT_EQ(snap1->left_stamp, Stamp(20));
  EXPECT_EQ(rclcpp::Time(snap1->left_info.header.stamp), Stamp(20));

  // Without a newer frame, a back-to-back call times out (even though
  // left_frame_ was moved out, last_left_stamp_ == 20 is preserved).
  auto snap2 =
      cache.WaitAndTakeSnapshot(absl::Milliseconds(30), absl::Milliseconds(30));
  EXPECT_EQ(snap2.status().code(), absl::StatusCode::kDeadlineExceeded);
}

TEST(FrameCacheTest, StereoRequiresMatchingTimestampsAndUpdatesInfoStamp) {
  FrameCache<FakeFrame> cache(/*stereo=*/true);
  cache.OnLeftInfo(MakeInfo(1));
  cache.OnRightInfo(MakeInfo(1));
  cache.SetWrapperReady(true);

  // Mismatched pair does not satisfy WaitAndTakeSnapshot.
  cache.OnLeftImage(std::make_unique<FakeFrame>(FakeFrame{10}), Stamp(100),
                    Stamp(100));
  cache.OnRightImage(std::make_unique<FakeFrame>(FakeFrame{9}), Stamp(99));
  EXPECT_FALSE(cache.HasFirstFrame());

  std::thread producer([&]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    cache.OnLeftImage(std::make_unique<FakeFrame>(FakeFrame{11}), Stamp(101),
                      Stamp(101));
    cache.OnRightImage(std::make_unique<FakeFrame>(FakeFrame{12}), Stamp(101));
  });
  auto snap = cache.WaitAndTakeSnapshot(absl::Seconds(1), absl::Seconds(1));
  producer.join();
  ASSERT_TRUE(snap.ok()) << snap.status();
  EXPECT_EQ(snap->left_frame->id, 11);
  EXPECT_EQ(snap->right_frame->id, 12);
  EXPECT_EQ(rclcpp::Time(snap->left_info.header.stamp), Stamp(101));
  EXPECT_EQ(rclcpp::Time(snap->right_info.header.stamp), Stamp(101));
}

TEST(FrameCacheTest, RestartDropsStaleFramesAndPreservesCameraInfo) {
  FrameCache<FakeFrame> cache(/*stereo=*/false);
  cache.OnLeftInfo(MakeInfo(1));
  cache.OnLeftImage(std::make_unique<FakeFrame>(FakeFrame{1}), Stamp(10),
                    Stamp(10));
  cache.SetWrapperReady(true);

  cache.BeginRestart();
  // CameraInfo remains available for Describe during restart.
  auto desc = cache.WaitForCameraInfo(absl::Milliseconds(50));
  ASSERT_TRUE(desc.ok()) << desc.status();
  EXPECT_EQ(desc->left_info.width, 1920u);

  // Queued pre-restart frame arriving during restart advances min_valid_stamp_.
  cache.OnLeftImage(std::make_unique<FakeFrame>(FakeFrame{2}), Stamp(15),
                    Stamp(15));
  cache.EndRestart(Stamp(16));

  // Late frame <= 15 is rejected; frame 16 is accepted.
  cache.OnLeftImage(std::make_unique<FakeFrame>(FakeFrame{3}), Stamp(15),
                    Stamp(16));
  EXPECT_FALSE(cache.HasFirstFrame());

  cache.OnLeftImage(std::make_unique<FakeFrame>(FakeFrame{4}), Stamp(16),
                    Stamp(16));
  EXPECT_TRUE(cache.HasFirstFrame());
  EXPECT_FALSE(cache.IsWrapperReady());

  // Snapshot initiated while restart finishes receives the first post-restart
  // frame once the wrapper thread marks readiness.
  std::thread ready_thread([&]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    cache.SetWrapperReady(true);
  });
  auto snap = cache.WaitAndTakeSnapshot(absl::Seconds(1), absl::Seconds(1));
  ready_thread.join();
  ASSERT_TRUE(snap.ok()) << snap.status();
  EXPECT_EQ(snap->left_frame->id, 4);
  EXPECT_EQ(rclcpp::Time(snap->left_info.header.stamp), Stamp(16));
}

TEST(FrameCacheTest, RequestStopWakesBlockedWaiters) {
  FrameCache<FakeFrame> cache(/*stereo=*/false);
  std::thread stopper([&]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    cache.RequestStop();
  });
  auto info = cache.WaitForCameraInfo(absl::Seconds(2));
  stopper.join();
  EXPECT_EQ(info.status().code(), absl::StatusCode::kUnavailable);
}

TEST(FrameCacheTest, LivenessSilenceDetection) {
  FrameCache<FakeFrame> cache(/*stereo=*/false);
  cache.ResetLivenessClock(Stamp(100));
  EXPECT_FALSE(cache.IsSilent(Stamp(115), rclcpp::Duration::from_seconds(15)));
  EXPECT_TRUE(cache.IsSilent(Stamp(116), rclcpp::Duration::from_seconds(15)));

  cache.BeginRestart();
  EXPECT_FALSE(cache.IsSilent(Stamp(200), rclcpp::Duration::from_seconds(15)));
  cache.EndRestart(Stamp(200));
  EXPECT_FALSE(cache.IsSilent(Stamp(205), rclcpp::Duration::from_seconds(15)));
}

}  // namespace
}  // namespace flowstate_zed
