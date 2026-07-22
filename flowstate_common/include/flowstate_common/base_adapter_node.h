/*
 * Copyright 2026 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef FLOWSTATE_COMMON_CAMERA_ADAPTER_NODE_H_
#define FLOWSTATE_COMMON_CAMERA_ADAPTER_NODE_H_

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "snapshot_interfaces/msg/sensor_info.hpp"
#include "snapshot_interfaces/srv/describe.hpp"
#include "snapshot_interfaces/srv/snapshot.hpp"

namespace flowstate_common {

/**
 * @class BaseAdapterNode
 * @brief Abstract base class for camera adapter nodes that implement the
 * Flowstate camera interface.
 *
 * This class provides common functionality for adapting ROS camera drivers to
 * work with Flowstate's snapshot interface. Derived classes should implement
 * camera-specific logic.
 *
 * Key responsibilities:
 * - Manage service handlers for "describe" and "snapshot" Flowstate services
 * - Subscribe to color image and camera info topics
 * - Manage a background thread for running the camera driver
 * - Provide liveness checking for connection monitoring
 *
 * To create a new camera adapter:
 * 1. Inherit from BaseAdapterNode
 * 2. Implement the pure virtual methods
 * 3. Subscribe to camera topics in the derived class constructor
 * 4. Override BuildDescribeResponse to build the describe response with sensor
 * information for the describe service
 * 5. Override BuildSnapshotResponse to build the snapshot response with image
 * data for the snapshot service
 *
 * Driver Initialization Note:
 * Derived classes have full control over their setup. Driver authors may introduce
 * custom helpers (e.g., `InitializeParameters()`) to handle ROS 2 parameters,
 * compile pipelines, or wrap SDK connections as needed. This heavy lifting
 * should generally be executed within the `Main()` background thread to avoid
 * blocking the SpawnerNode.
 */
class BaseAdapterNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor initializes the adapter node with serial and locators.
   * @param serial Camera serial number
   * @param locators Camera locators (IP addresses should come first after that,
   * USB paths)
   * @param node_name_prefix ROS node name prefix (will be prefixed with camera
   * type)
   */
  BaseAdapterNode(
      const std::string& serial, const std::vector<std::string>& locators,
      const std::string& node_name_prefix,
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  virtual ~BaseAdapterNode();

  /**
   * @brief Check if the background thread has exited.
   * @return true if thread has exited, false otherwise
   */
  bool HasExitedThread() const { return exited_thread_.load(); }

  /**
   * @brief Get the camera serial number.
   * @return Camera serial number
   */
  std::string GetSerial() const { return serial_; }

 protected:
  /**
   * @brief Main executor loop. Derived classes should override to run their
   * camera driver's executor.
   * @return Status of the main loop execution
   */
  virtual absl::Status Main() = 0;

  /**
   * @brief Get the color image topic name for this camera.
   * Must be implemented by derived classes.
   * @return Topic name string (e.g., "/camera_0/color/image_raw")
   */
  virtual std::string ColorImageTopic() const = 0;

  /**
   * @brief Build the response for the describe service.
   * Derived classes should override to provide camera-specific sensor
   * information.
   * @return The populated response, or an error status on failure.
   */
  virtual absl::StatusOr<snapshot_interfaces::srv::Describe::Response>
  BuildDescribeResponse() = 0;

  /**
   * @brief Build the response for the snapshot service.
   * Derived classes should override to handle multiple image types (IR, depth,
   * etc).
   * @return The populated response, or an error status on failure.
   */
  virtual absl::StatusOr<snapshot_interfaces::srv::Snapshot::Response>
  BuildSnapshotResponse() = 0;

  /**
   * @brief Get color camera info.
   * Thread-safe access to cached color camera info.
   * @return unique_ptr to camera info (nullptr if not yet received)
   */
  std::unique_ptr<sensor_msgs::msg::CameraInfo> GetColorCameraInfo() const
      ABSL_LOCKS_EXCLUDED(camera_info_mutex_) {
    absl::MutexLock lock(&camera_info_mutex_);
    if (!color_camera_info_) return nullptr;
    return std::make_unique<sensor_msgs::msg::CameraInfo>(*color_camera_info_);
  }

  /**
   * @brief Get color image.
   * Thread-safe access to cached color image.
   * @return unique_ptr to image (nullptr if not yet received)
   */
  std::unique_ptr<sensor_msgs::msg::Image> GetColorImage() const
      ABSL_LOCKS_EXCLUDED(image_mutex_) {
    absl::MutexLock lock(&image_mutex_);
    if (!color_image_) return nullptr;
    return std::make_unique<sensor_msgs::msg::Image>(*color_image_);
  }

  /**
   * @brief Helper to create the standard Flowstate service handlers.
   * Call this in derived class constructor after subscribing to all topics.
   */
  void CreateFlowstateServices();

  /**
   * @brief Start the background executor thread.
   * Call this in derived class constructor after all subscriptions are set up.
   */
  void StartExecutorThread();

  /**
   * @brief Helper to pack CameraInfo into the Response.
   * Call this from BuildDescribeResponse() in derived classes to add a sensor
   * description. This form populates camera_t_sensor with a unity transform.
   * @param camera_info CameraInfo message to extract sensor parameters from
   * @param sensor_name Logical name for the sensor (e.g., "rgb")
   * @param topic_name ROS topic name for the sensor's image stream
   * @return SensorInfo message populated with the provided info and parameters
   */
  snapshot_interfaces::msg::SensorInfo BuildSensorInformation(
      const sensor_msgs::msg::CameraInfo& camera_info,
      const std::string& sensor_name, const std::string& topic_name);

  /**
   * @brief Helper to pack CameraInfo into the Response.
   * Call this from BuildDescribeResponse() in derived classes to add a sensor
   * description. This form accepts a camera_t_sensor transform.
   * @param camera_info CameraInfo message to extract sensor parameters from
   * @param sensor_name Logical name for the sensor (e.g., "rgb")
   * @param topic_name ROS topic name for the sensor's image stream
   * @return SensorInfo message populated with the provided info and parameters
   */
  snapshot_interfaces::msg::SensorInfo BuildSensorInformation(
      const sensor_msgs::msg::CameraInfo& camera_info,
      const std::string& sensor_name, const std::string& topic_name,
      const geometry_msgs::msg::TransformStamped& camera_t_sensor);

  // Thread management
  std::thread thread_;
  std::atomic<bool> exited_thread_{false};

  // Camera identification
  std::string serial_;
  std::vector<std::string> locators_;

  // Subscriptions and cached data
  mutable absl::Mutex camera_info_mutex_;
  std::unique_ptr<sensor_msgs::msg::CameraInfo> color_camera_info_
      ABSL_GUARDED_BY(camera_info_mutex_);
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_sub_;

  mutable absl::Mutex image_mutex_;
  std::unique_ptr<sensor_msgs::msg::Image> color_image_
      ABSL_GUARDED_BY(image_mutex_);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;

  // Timeout monitoring
  mutable absl::Mutex timeout_mutex_;
  rclcpp::Time t_last_color_image_ ABSL_GUARDED_BY(timeout_mutex_);
  rclcpp::TimerBase::SharedPtr liveness_timer_;

  // Flowstate services
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Service<snapshot_interfaces::srv::Describe>::SharedPtr
      describe_service_;
  rclcpp::Service<snapshot_interfaces::srv::Snapshot>::SharedPtr
      snapshot_service_;

 private:
  void DescribeCallback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<snapshot_interfaces::srv::Describe::Request>
          request,
      const std::shared_ptr<snapshot_interfaces::srv::Describe::Response>
          response);

  void SnapshotCallback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<snapshot_interfaces::srv::Snapshot::Request>
          request,
      const std::shared_ptr<snapshot_interfaces::srv::Snapshot::Response>
          response);
};

}  // namespace flowstate_common

#endif  // FLOWSTATE_COMMON_CAMERA_ADAPTER_NODE_H_
