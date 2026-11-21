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

#ifndef FLOWSTATE_ZED_FRAME_CACHE_H_
#define FLOWSTATE_ZED_FRAME_CACHE_H_

#include <algorithm>
#include <memory>
#include <optional>
#include <utility>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace flowstate_zed {

// Thread-safe cache for raw camera frames, CameraInfo, restart stamp-gating,
// stereo timestamp matching, and liveness tracking.
//
// Frames are moved out on snapshot (O(1) pointer move under lock), while
// `last_left_stamp_` is preserved so back-to-back snapshots still wait for a
// strictly newer frame.
template <typename FrameT>
class FrameCache {
 public:
  struct DescribeData {
    sensor_msgs::msg::CameraInfo left_info;
    std::optional<sensor_msgs::msg::CameraInfo> right_info;
  };

  struct SnapshotData {
    std::unique_ptr<FrameT> left_frame;
    rclcpp::Time left_stamp;
    sensor_msgs::msg::CameraInfo left_info;
    std::unique_ptr<FrameT> right_frame;
    sensor_msgs::msg::CameraInfo right_info;
  };

  explicit FrameCache(bool stereo) : stereo_(stereo) {}

  void OnLeftImage(std::unique_ptr<FrameT> frame, const rclcpp::Time& stamp,
                   const rclcpp::Time& received_at) {
    absl::MutexLock lock(&mutex_);
    if (!AcceptStampLocked(stamp)) return;
    left_frame_ = StampedFrame{std::move(frame), stamp};
    last_left_stamp_ = stamp;
    last_left_received_at_ = received_at;
  }

  void OnRightImage(std::unique_ptr<FrameT> frame, const rclcpp::Time& stamp) {
    absl::MutexLock lock(&mutex_);
    if (!AcceptStampLocked(stamp)) return;
    right_frame_ = StampedFrame{std::move(frame), stamp};
  }

  void OnLeftInfo(std::unique_ptr<sensor_msgs::msg::CameraInfo> info) {
    absl::MutexLock lock(&mutex_);
    left_info_ = std::move(info);
  }

  void OnRightInfo(std::unique_ptr<sensor_msgs::msg::CameraInfo> info) {
    absl::MutexLock lock(&mutex_);
    right_info_ = std::move(info);
  }

  void ResetLivenessClock(const rclcpp::Time& now) {
    absl::MutexLock lock(&mutex_);
    last_left_received_at_ = now;
  }

  // Starts a wrapper restart: marks restarting so incoming pre-restart frames
  // advance `min_valid_stamp_` and are dropped, clears cached frames, and
  // preserves CameraInfo so Describe stays responsive.
  void BeginRestart() {
    absl::MutexLock lock(&mutex_);
    restarting_ = true;
    wrapper_ready_ = false;
    if (left_frame_) {
      AdvanceMinValidStampLocked(left_frame_->stamp);
      left_frame_.reset();
    }
    if (right_frame_) {
      AdvanceMinValidStampLocked(right_frame_->stamp);
      right_frame_.reset();
    }
    if (last_left_stamp_) {
      AdvanceMinValidStampLocked(*last_left_stamp_);
      last_left_stamp_.reset();
    }
  }

  void EndRestart(const rclcpp::Time& now) {
    absl::MutexLock lock(&mutex_);
    last_left_received_at_ = now;
    restarting_ = false;
  }

  void SetWrapperReady(bool ready) {
    absl::MutexLock lock(&mutex_);
    wrapper_ready_ = ready;
  }

  bool IsWrapperReady() const {
    absl::MutexLock lock(&mutex_);
    return wrapper_ready_;
  }

  void RequestStop() {
    absl::MutexLock lock(&mutex_);
    stop_requested_ = true;
  }

  bool IsStopRequested() const {
    absl::MutexLock lock(&mutex_);
    return stop_requested_;
  }

  // Returns true once at least one valid frame (or stamp-matched stereo pair)
  // and matching CameraInfo have arrived.
  bool HasFirstFrame() const {
    absl::MutexLock lock(&mutex_);
    if (!left_frame_ || !left_info_) return false;
    if (!stereo_) return true;
    return right_frame_ && right_info_ &&
           left_frame_->stamp == right_frame_->stamp;
  }

  // Returns true if no color frame has arrived for more than `timeout` while
  // not restarting.
  bool IsSilent(const rclcpp::Time& now, const rclcpp::Duration& timeout,
                rclcpp::Duration* silent_for = nullptr) const {
    absl::MutexLock lock(&mutex_);
    if (restarting_ || !last_left_received_at_) return false;
    const rclcpp::Duration elapsed = now - *last_left_received_at_;
    if (silent_for != nullptr) {
      *silent_for = elapsed;
    }
    return elapsed > timeout;
  }

  absl::StatusOr<DescribeData> WaitForCameraInfo(absl::Duration deadline) {
    absl::MutexLock lock(&mutex_);
    const auto has_info = [this]() ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
      return stop_requested_ ||
             (left_info_ != nullptr && (!stereo_ || right_info_ != nullptr));
    };
    const bool ready =
        mutex_.AwaitWithTimeout(absl::Condition(&has_info), deadline);
    if (stop_requested_) {
      return absl::UnavailableError("Adapter stopping");
    }
    if (!ready) {
      return absl::UnavailableError("CameraInfo not yet received");
    }
    DescribeData data;
    data.left_info = *left_info_;
    if (stereo_) {
      data.right_info = *right_info_;
    }
    return data;
  }

  // Waits for a frame (or stamp-matched stereo pair) strictly newer than the
  // last frame seen before the call, then moves the frame(s) out.
  absl::StatusOr<SnapshotData> WaitAndTakeSnapshot(
      absl::Duration image_deadline, absl::Duration startup_deadline) {
    absl::MutexLock lock(&mutex_);
    std::optional<rclcpp::Time> stamp_at_call;
    if (!restarting_ && wrapper_ready_ && last_left_stamp_) {
      stamp_at_call = last_left_stamp_;
    }
    const absl::Duration deadline =
        stamp_at_call ? image_deadline : startup_deadline;
    const auto fresh = [this,
                        stamp_at_call]() ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
      if (stop_requested_) return true;
      if (restarting_ || !wrapper_ready_) return false;
      if (!left_frame_ || !left_info_) return false;
      if (stamp_at_call && left_frame_->stamp <= *stamp_at_call) {
        return false;
      }
      if (!stereo_) return true;
      return right_frame_ && right_info_ &&
             left_frame_->stamp == right_frame_->stamp;
    };
    const bool ready =
        mutex_.AwaitWithTimeout(absl::Condition(&fresh), deadline);
    if (stop_requested_) {
      return absl::UnavailableError("Adapter stopping");
    }
    if (!ready) {
      return absl::DeadlineExceededError(
          absl::StrCat("No new frame within ", absl::FormatDuration(deadline)));
    }

    SnapshotData out;
    out.left_frame = std::move(left_frame_->frame);
    out.left_stamp = left_frame_->stamp;
    out.left_info = *left_info_;
    out.left_info.header.stamp = out.left_stamp;
    left_frame_.reset();
    if (stereo_) {
      out.right_frame = std::move(right_frame_->frame);
      out.right_info = *right_info_;
      out.right_info.header.stamp = out.left_stamp;
      right_frame_.reset();
    }
    return out;
  }

 private:
  struct StampedFrame {
    std::unique_ptr<FrameT> frame;
    rclcpp::Time stamp;
  };

  void AdvanceMinValidStampLocked(const rclcpp::Time& stamp)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
    min_valid_stamp_ =
        min_valid_stamp_ ? std::max(*min_valid_stamp_, stamp) : stamp;
  }

  bool AcceptStampLocked(const rclcpp::Time& stamp)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
    if (restarting_) {
      AdvanceMinValidStampLocked(stamp);
      return false;
    }
    if (min_valid_stamp_ && stamp <= *min_valid_stamp_) {
      return false;
    }
    return true;
  }

  const bool stereo_;
  mutable absl::Mutex mutex_;
  std::optional<StampedFrame> left_frame_ ABSL_GUARDED_BY(mutex_);
  std::optional<StampedFrame> right_frame_ ABSL_GUARDED_BY(mutex_);
  std::optional<rclcpp::Time> last_left_stamp_ ABSL_GUARDED_BY(mutex_);
  std::optional<rclcpp::Time> last_left_received_at_ ABSL_GUARDED_BY(mutex_);
  std::unique_ptr<sensor_msgs::msg::CameraInfo> left_info_
      ABSL_GUARDED_BY(mutex_);
  std::unique_ptr<sensor_msgs::msg::CameraInfo> right_info_
      ABSL_GUARDED_BY(mutex_);
  std::optional<rclcpp::Time> min_valid_stamp_ ABSL_GUARDED_BY(mutex_);
  bool wrapper_ready_ ABSL_GUARDED_BY(mutex_) = false;
  bool restarting_ ABSL_GUARDED_BY(mutex_) = false;
  bool stop_requested_ ABSL_GUARDED_BY(mutex_) = false;
};

}  // namespace flowstate_zed

#endif  // FLOWSTATE_ZED_FRAME_CACHE_H_
