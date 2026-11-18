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

#ifndef FLOWSTATE_ZED_ADAPTER_NODE_H_
#define FLOWSTATE_ZED_ADAPTER_NODE_H_

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "flowstate_common/base_adapter_node.h"
#include "flowstate_zed/camera_discovery.h"
#include "flowstate_zed/frame_cache.h"
#include "flowstate_zed/zed_parameters.h"
#include "geometry_msgs/msg/transform.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "snapshot_interfaces/srv/describe.hpp"
#include "snapshot_interfaces/srv/snapshot.hpp"
#include "zed_components/sl_types.hpp"

namespace flowstate_zed {

// Adapts one ZED X camera to the Flowstate snapshot interface. Owns an
// in-process Stereolabs wrapper component and receives raw frames via
// SlImageAdapter intra-process pub/sub.
class AdapterNode : public flowstate_common::BaseAdapterNode {
 public:
  explicit AdapterNode(const DiscoveredCamera& camera);
  ~AdapterNode() override;

  const DiscoveredCamera& GetCamera() const { return camera_; }

 private:
  // Setup.
  void DeclareParameters();
  void Subscribe();
  std::string ColorImageTopic() const override;

  // Adapter and wrapper lifecycle.
  absl::Status Main() override;
  absl::Status StartWrapper() ABSL_LOCKS_EXCLUDED(wrapper_mutex_);
  void StopWrapper() ABSL_LOCKS_EXCLUDED(wrapper_mutex_);

  // Flowstate service handlers.
  absl::StatusOr<snapshot_interfaces::srv::Describe::Response>
  BuildDescribeResponse() override;
  absl::StatusOr<snapshot_interfaces::srv::Snapshot::Response>
  BuildSnapshotResponse() override;

  // Dynamic parameter updates.
  void PostSetParameters(const std::vector<rclcpp::Parameter>& parameters);
  void SyncWrapperParameters() ABSL_LOCKS_EXCLUDED(wrapper_mutex_);
  void ApplyVideoSettingsToWrapper(rclcpp::Node& wrapper,
                                   const CameraSettings& settings)
      ABSL_LOCKS_EXCLUDED(pending_mutex_);

  const DiscoveredCamera camera_;

  // Lock order: NodeParameters::mutex_ (inside rclcpp) -> wrapper_mutex_ ->
  // {pending_mutex_, FrameCache::mutex_}. `pending_mutex_`,
  // `FrameCache::mutex_`, and `executor_mutex_` are leaf locks and never
  // acquire another mutex.
  absl::Mutex wrapper_mutex_;
  std::shared_ptr<rclcpp::Node> wrapper_ ABSL_GUARDED_BY(wrapper_mutex_);
  std::thread wrapper_thread_ ABSL_GUARDED_BY(wrapper_mutex_);
  int64_t wrapper_fps_ ABSL_GUARDED_BY(wrapper_mutex_) = 0;
  std::atomic<bool> wrapper_stop_{false};

  // Coordinates video settings written while the wrapper is starting up:
  // `wrapper_thread_` drains `pending_settings_` and sets
  // `wrapper_accepting_sync_` before advertising
  // `cache_.SetWrapperReady(true)`.
  absl::Mutex pending_mutex_;
  bool wrapper_accepting_sync_ ABSL_GUARDED_BY(pending_mutex_) = false;
  std::optional<CameraSettings> pending_settings_
      ABSL_GUARDED_BY(pending_mutex_);
  StereoAutoRangeState stereo_auto_range_ ABSL_GUARDED_BY(pending_mutex_);

  absl::Mutex executor_mutex_;
  rclcpp::Executor* active_executor_ ABSL_GUARDED_BY(executor_mutex_) = nullptr;

  FrameCache<stereolabs::StampedSlMat> cache_;
  // Written once in Main() before StartWrapper() and read-only afterward.
  geometry_msgs::msg::Transform right_pose_;

  rclcpp::CallbackGroup::SharedPtr subscription_group_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::Subscription<SlImageAdapter>::SharedPtr left_image_sub_;
  rclcpp::Subscription<SlImageAdapter>::SharedPtr right_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_sub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      on_set_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
      post_set_handle_;
};

}  // namespace flowstate_zed

#endif  // FLOWSTATE_ZED_ADAPTER_NODE_H_
