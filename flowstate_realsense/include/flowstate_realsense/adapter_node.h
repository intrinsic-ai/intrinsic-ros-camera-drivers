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

#ifndef FLOWSTATE_REALSENSE_ADAPTER_NODE_H_
#define FLOWSTATE_REALSENSE_ADAPTER_NODE_H_

#include <array>
#include <atomic>
#include <cstddef>
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
#include "class_loader/class_loader.hpp"
#include "flowstate_common/base_adapter_node.h"
#include "flowstate_realsense/helpers.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/parameter_service.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"
#include "rclcpp_components/node_instance_wrapper.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "snapshot_interfaces/srv/describe.hpp"
#include "snapshot_interfaces/srv/snapshot.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace flowstate_realsense {

// One RealSense camera for Flowstate: realsense2_camera's node composed
// in-process (loaded as the rclcpp component it registers, so none of the
// vendor's headers are needed here; namespace /realsense, name and
// camera_name camera_<serial>), its color, aligned-depth and two infrared
// topics cached here, and the Flowstate describe and snapshot services on this
// node, named realsense_<serial>.
//
// The camera's settings are the vendor node's parameters: this node's
// parameter services serve the vendor node's parameters, so a Flowstate
// CameraSetting names a realsense2_camera parameter verbatim (enable_infra1,
// depth_module.exposure, ...), with the vendor's types, ranges and live values.
//
// Four sensors, described in this fixed order (Flowstate numbers them 1..4):
// rgb, depth (aligned to color and converted from the driver's 16-bit
// millimeters to 32-bit float meters, the one depth encoding Flowstate's ROS
// image source accepts), ir_left and ir_right (Y8, mono8). The camera frame is
// the color optical frame: color and aligned depth report camera_t_sensor =
// identity, the infrared sensors their static transforms. Every stream starts
// on, so the describe Flowstate takes when it opens the camera holds all four
// sensors; a snapshot returns those the vendor's enable_* parameters leave on.
class AdapterNode : public flowstate_common::BaseAdapterNode {
 public:
  explicit AdapterNode(const std::string& serial);
  ~AdapterNode() override;

  static constexpr size_t kStreamCount = 4;

 private:
  // One stream's latest messages. The CameraInfo and camera_t_sensor stay
  // after the stream is turned off, so the describe keeps all four sensors.
  // Images and CameraInfo are stored as shared_ptr<const T> (zero-copy promoted
  // from intra-process UniquePtr) so snapshots copy only pointers under mutex_.
  struct StreamState {
    std::shared_ptr<const sensor_msgs::msg::CameraInfo> info;
    std::shared_ptr<const sensor_msgs::msg::Image> image;
    std::optional<geometry_msgs::msg::TransformStamped> camera_t_sensor;
    // A snapshot waits for frames counted after the request. t_last_frame and
    // t_last_info are arrival times on this node's clock: the liveness check
    // and settings gate read them, never the vendor's header stamps, which map
    // the camera's clock onto a time base set once at the first frame.
    uint64_t frame_count = 0;
    rclcpp::Time t_last_frame;
    rclcpp::Time t_last_info;
    // Whether the stream was served at the last liveness check: its silence
    // counts from the moment it was turned (back) on.
    bool served = false;
  };

  absl::Status Main() override ABSL_LOCKS_EXCLUDED(mutex_, restart_mutex_);
  std::string ColorImageTopic() const override;
  std::string ImageTopic(size_t stream) const;
  std::string InfoTopic(size_t stream) const;
  std::string CameraName() const;
  absl::StatusOr<snapshot_interfaces::srv::Describe::Response>
  BuildDescribeResponse() override ABSL_LOCKS_EXCLUDED(mutex_);
  absl::StatusOr<snapshot_interfaces::srv::Snapshot::Response>
  BuildSnapshotResponse() override ABSL_LOCKS_EXCLUDED(mutex_, restart_mutex_);

  // The parameters realsense2_camera starts with: this serial, every stream
  // on (Y8 infrared), depth aligned to color, static transforms only.
  std::vector<rclcpp::Parameter> StartupParameters() const;
  // Loads realsense2_camera's component for this serial, spins it on its own
  // executor thread and serves its parameters on this node. Called from Main().
  absl::Status StartDriver();
  // Tears down the composed vendor node in strict unload order so the vendor
  // library is unloaded only after all weak references and threads are gone.
  void StopDriver();
  void Subscribe(size_t stream);
  // True once realsense2_camera has opened the device and declared its stream
  // and filter parameters. Must be called outside mutex_.
  bool DeviceOpened() const ABSL_LOCKS_EXCLUDED(mutex_);
  // Which streams the vendor's parameters leave on, in sensor order; none
  // before DeviceOpened(). Aligned depth also needs enable_color and
  // align_depth.enable. Reads vendor parameters outside mutex_.
  std::array<bool, kStreamCount> ServedStreams() const
      ABSL_LOCKS_EXCLUDED(mutex_);
  // Expected output dimensions per stream parsed from the vendor's active
  // *_profile parameters (depth aligns to color so it shares color's size).
  // Reads vendor parameters outside mutex_.
  std::array<std::optional<ImageDimensions>, kStreamCount> ExpectedStreamSizes()
      const ABSL_LOCKS_EXCLUDED(mutex_);
  // realsense-ros only stores a changed profile or format until the stream
  // restarts: re-sets enable_color to its own value, which notifies the vendor
  // to restart every sensor whose wanted profiles no longer match. Called only
  // from BuildSnapshotResponse() so batched profile edits do not trigger
  // mid-batch restarts. Serialized by restart_mutex_.
  void RestartStreamsIfPending() ABSL_LOCKS_EXCLUDED(mutex_, restart_mutex_);
  // The 1 s housekeeping: camera_t_sensor for each stream whose images have
  // arrived, and the liveness check over the served streams.
  void CacheExtrinsics() ABSL_LOCKS_EXCLUDED(mutex_);
  void CheckLiveness() ABSL_LOCKS_EXCLUDED(mutex_);

  // This node's executor, run by Main() on the base class's thread. A member
  // so the destructor can cancel it: the spawner also deletes a node whose
  // camera was unplugged while Main() still spins, and the base class joins
  // the thread without cancelling anything. Four threads: the two services,
  // the subscriptions and the default group (the parameter services and the
  // housekeeping timer) share them.
  rclcpp::executors::MultiThreadedExecutor executor_{rclcpp::ExecutorOptions(),
                                                     4};

  // Set by the destructor or liveness timeout to end Main() and wake any
  // snapshot waiter immediately.
  std::atomic<bool> stop_requested_{false};

  // The composed vendor node: the loader must outlive the factory, which must
  // outlive the instance and its executor, which holds weak references to the
  // instance's entities (whose control blocks are the library's code);
  // StopDriver() releases them in that order.
  std::unique_ptr<class_loader::ClassLoader> driver_loader_;
  std::shared_ptr<rclcpp_components::NodeFactory> driver_factory_;
  std::unique_ptr<rclcpp_components::NodeInstanceWrapper> driver_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> driver_executor_;
  std::thread driver_thread_;
  // The vendor node as an rclcpp::Node, its parameters served as this node's
  // (the proxy holds the vendor's parameters interface), and the callback that
  // stamps each committed change of them.
  std::shared_ptr<rclcpp::Node> vendor_;
  std::shared_ptr<rclcpp::ParameterService> parameter_service_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
      settings_changed_handle_;

  // Serializes stream restarts across concurrent Snapshot calls.
  // Lock order: restart_mutex_ -> vendor_ parameters -> mutex_.
  absl::Mutex restart_mutex_;

  // The static transforms realsense2_camera publishes for its active streams,
  // until all stream extrinsics are cached.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::CallbackGroup::SharedPtr subscription_cb_group_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;

  std::array<StreamState, kStreamCount> streams_ ABSL_GUARDED_BY(mutex_);
  // Construction time: the liveness check counts the driver's silence from it
  // until the device is open.
  const rclcpp::Time t_started_;
  // The last committed change of the vendor's parameters: a snapshot skips
  // frames received before it (exposed under the previous settings).
  rclcpp::Time t_settings_changed_ ABSL_GUARDED_BY(mutex_);
  // A profile or format changed since the streams last restarted.
  bool restart_pending_ ABSL_GUARDED_BY(mutex_) = false;
};

}  // namespace flowstate_realsense

#endif  // FLOWSTATE_REALSENSE_ADAPTER_NODE_H_
