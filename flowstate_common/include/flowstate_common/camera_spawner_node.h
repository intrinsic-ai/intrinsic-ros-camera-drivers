#ifndef FLOWSTATE_COMMON_CAMERA_SPAWNER_NODE_H_
#define FLOWSTATE_COMMON_CAMERA_SPAWNER_NODE_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "rclcpp/rclcpp.hpp"
#include "snapshot_interfaces/msg/discovered_camera.hpp"
#include "snapshot_interfaces/srv/discover.hpp"

namespace flowstate_common {

/**
 * @class CameraSpawnerNode
 * @brief Abstract base class for camera spawner nodes that discover and manage
 * camera adapter nodes.
 *
 * This class handles the common discovery and management tasks for camera
 * drivers. Derived classes should implement camera-specific discovery logic.
 *
 * Key responsibilities:
 * - Periodically discover connected cameras
 * - Spawn adapter nodes for new cameras
 * - Provide the "discover" service for Flowstate
 * - Clean up nodes when cameras disconnect
 *
 * To create a new camera spawner:
 * 1. Inherit from CameraSpawnerNode
 * 2. Implement UpdateCameraList() with camera-specific discovery logic
 * 3. Implement IsAlreadySpawned() to check for existing nodes
 * 4. Call RegisterDiscoveredCamera() for each discovered camera
 * 5. Call CreateAdapterNode() to spawn the adapter for each camera
 */
class CameraSpawnerNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor initializes the spawner node.
   * @param node_name ROS node name
   * @param discovery_interval_seconds How often to rediscover cameras
   */
  CameraSpawnerNode(const std::string& node_name,
                    double discovery_interval_seconds = 5.0);

  virtual ~CameraSpawnerNode();

 protected:
  /**
   * @brief Update the list of discovered cameras.
   * Derived classes should implement this to discover cameras specific to their
   * hardware.
   *
   * This method should:
   * 1. Query for connected cameras
   * 2. Call RegisterDiscoveredCamera() for each found camera
   * 3. Call CreateAdapterNode() to spawn adapter nodes for new cameras
   */
  virtual void UpdateCameraList() = 0;

  /**
   * @brief Check if an adapter node already exists for the given serial.
   * Derived classes should implement this to check their spawned nodes.
   * @param serial Camera serial number
   * @return true if adapter already exists, false otherwise
   */
  virtual bool IsAlreadySpawned(const std::string& serial) const = 0;

  /**
   * @brief Get the serial number for a discovered camera.
   * Derived classes should implement this to extract serial from their
   * discovery data.
   * @return Camera serial number
   */
  virtual std::string GetDiscoveredCameraSerial(size_t index) const = 0;

  /**
   * @brief Get the IP address for a discovered camera (if applicable).
   * Default implementation returns empty string (for USB devices).
   * @return Camera IP address or empty string
   */
  virtual std::string GetDiscoveredCameraIp(size_t index) const {
    return "";
  }

  /**
   * @brief Register a discovered camera.
   * Call this during UpdateCameraList() for each discovered camera.
   * @param serial Camera serial number
   * @param model Camera model/type string
   * @param ip_address Camera IP address (empty for USB)
   */
  void RegisterDiscoveredCamera(const std::string& serial,
                                const std::string& model,
                                const std::string& ip_address = "");

  /**
   * @brief Get list of discovered camera serials.
   * @return Vector of camera serial numbers
   */
  std::vector<std::string> GetDiscoveredSerials() const {
    absl::MutexLock lock(&cameras_mutex_);
    std::vector<std::string> serials;
    for (const auto& camera : discovered_cameras_) {
      serials.push_back(camera.serial);
    }
    return serials;
  }

  /**
   * @brief Get the discovered cameras message.
   * @return Vector of DiscoveredCamera messages
   */
  std::vector<snapshot_interfaces::msg::DiscoveredCamera>
  GetDiscoveredCameras() const {
    absl::MutexLock lock(&cameras_mutex_);
    return discovered_camera_msgs_;
  }

  // Protected members for derived classes to access
  mutable absl::Mutex cameras_mutex_;
  std::vector<snapshot_interfaces::msg::DiscoveredCamera>
      discovered_camera_msgs_ ABSL_GUARDED_BY(cameras_mutex_);

 private:
  struct DiscoveredCameraInfo {
    std::string serial;
    std::string model;
    std::string ip_address;
  };

  void DiscoveryTimer();

  void DiscoverServiceCallback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<snapshot_interfaces::srv::Discover::Request>
          request,
      const std::shared_ptr<snapshot_interfaces::srv::Discover::Response>
          response);

  rclcpp::Service<snapshot_interfaces::srv::Discover>::SharedPtr
      discover_service_;
  rclcpp::TimerBase::SharedPtr discovery_timer_;

  mutable absl::Mutex discovery_mutex_;
  std::vector<DiscoveredCameraInfo> discovered_cameras_
      ABSL_GUARDED_BY(discovery_mutex_);
};

}  // namespace flowstate_common

#endif  // FLOWSTATE_COMMON_CAMERA_SPAWNER_NODE_H_
