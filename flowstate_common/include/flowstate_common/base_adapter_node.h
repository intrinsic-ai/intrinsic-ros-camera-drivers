#ifndef FLOWSTATE_COMMON_CAMERA_ADAPTER_NODE_H_
#define FLOWSTATE_COMMON_CAMERA_ADAPTER_NODE_H_

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "snapshot_interfaces/msg/image_snapshot.hpp"
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
 */
class BaseAdapterNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor initializes the adapter node with serial and IP address.
   * @param serial Camera serial number
   * @param ip_address Camera IP address (optional for USB devices)
   * @param node_name ROS node name prefix (will be prefixed with camera type)
   */
  BaseAdapterNode(const std::string& serial, const std::string& ip_address,
                  const std::string& node_name_prefix);

  virtual ~BaseAdapterNode();

  /**
   * @brief Check if the background thread has exited.
   * @return true if thread has exited, false otherwise
   */
  bool HasExitedThread() const { return exited_thread_; }

  /**
   * @brief Check if this adapter is managing the specified camera serial.
   * @param serial Camera serial number to check
   * @return true if serials match, false otherwise
   */
  bool HasSerial(const std::string& serial) const { return serial_ == serial; }

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
   * @param response Describe service response to populate
   * @return true on success, false on error (caller will set error_message)
   */
  virtual bool BuildDescribeResponse(
      snapshot_interfaces::srv::Describe::Response& response) = 0;

  /**
   * @brief Build the response for the snapshot service.
   * Derived classes should override to handle multiple image types (IR, depth,
   * etc).
   * @param response Snapshot service response to populate
   * @return true on success, false on error (caller will set error_message)
   */
  virtual bool BuildSnapshotResponse(
      snapshot_interfaces::srv::Snapshot::Response& response) = 0;

  /**
   * @brief Get color camera info.
   * Thread-safe access to cached color camera info.
   * @return shared_ptr to camera info (nullptr if not yet received)
   */
  std::unique_ptr<sensor_msgs::msg::CameraInfo> GetColorCameraInfo() const {
    absl::MutexLock lock(&camera_info_mutex_);
    if (!color_camera_info_) return nullptr;
    return std::make_unique<sensor_msgs::msg::CameraInfo>(*color_camera_info_);
  }

  /**
   * @brief Get color image.
   * Thread-safe access to cached color image.
   * @return shared_ptr to image (nullptr if not yet received)
   */
  std::unique_ptr<sensor_msgs::msg::Image> GetColorImage() const {
    absl::MutexLock lock(&image_mutex_);
    if (!color_image_) return nullptr;
    return std::make_unique<sensor_msgs::msg::Image>(*color_image_);
  }

  /**
   * @brief Subscribe to camera info topic.
   * Helper for derived classes to subscribe to camera info.
   */
  template <typename Func>
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
  SubscribeToCameraInfo(const std::string& topic, Func callback) {
    return create_subscription<sensor_msgs::msg::CameraInfo>(topic, 2,
                                                             callback);
  }

  /**
   * @brief Subscribe to image topic.
   * Helper for derived classes to subscribe to images.
   */
  template <typename Func>
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr SubscribeToImage(
      const std::string& topic, Func callback) {
    return create_subscription<sensor_msgs::msg::Image>(topic, 2, callback);
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
   * description for the color camera.
   * @param response Describe service response to populate
   * @param info CameraInfo message to extract sensor parameters from
   * @param sensor_name Logical name for the sensor (e.g., "rgb")
   * @param topic_name ROS topic name for the sensor's image stream
   */
  void AppendSensorDescription(
      snapshot_interfaces::srv::Describe::Response& response,
      const sensor_msgs::msg::CameraInfo& info, const std::string& sensor_name,
      const std::string& topic_name);

  // Thread management
  std::thread thread_;
  bool exited_thread_ = false;

  // Camera identification
  std::string serial_;
  std::string ip_address_;

  // Subscriptions and cached data
  mutable absl::Mutex camera_info_mutex_;
  std::unique_ptr<sensor_msgs::msg::CameraInfo> color_camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_sub_;

  mutable absl::Mutex image_mutex_;
  std::unique_ptr<sensor_msgs::msg::Image> color_image_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;

  // Timeout monitoring
  mutable absl::Mutex timeout_mutex_;
  rclcpp::Time t_last_color_image_;
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
