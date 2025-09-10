#include "flowstate_orbbec/spawner_node.h"

#include <memory>

#include "orbbec_camera/ob_camera_node_driver.h"
#include "rclcpp/rclcpp.hpp"
#include "snapshot_interfaces/msg/discovered_camera.hpp"
#include "snapshot_interfaces/srv/discover.hpp"

namespace flowstate_orbbec {

using snapshot_interfaces::srv::Discover;

SpawnerNode::SpawnerNode()
    : Node("flowstate_orbbec") {
  discover_service_ = create_service<Discover>(
      "/cameras/discover",
      [this](const std::shared_ptr<rmw_request_id_t>,
             const std::shared_ptr<Discover::Request>,
             const std::shared_ptr<Discover::Response> response) {
        RCLCPP_INFO(get_logger(), "Discover service called");
        absl::MutexLock lock(&this->serials_mutex_);
        for (const std::string& serial : serials_) {
          snapshot_interfaces::msg::DiscoveredCamera camera;
          camera.driver_type = "orbbec";
          camera.camera_id = serial;
          response->cameras.push_back(camera);
        }
        response->success = true;
      });
  timer_ = create_wall_timer(std::chrono::seconds(10),
                             [this]() { this->UpdateCameras(); });
}

void SpawnerNode::UpdateCameras() {
  // ob::Context::setLoggerSeverity(OBLogSeverity::OB_LOG_SEVERITY_OFF);
  auto context = std::make_unique<ob::Context>();
  auto list = context->queryDeviceList();
  absl::MutexLock lock(&this->serials_mutex_);
  serials_.clear();
  for (size_t i = 0; i < list->deviceCount(); i++) {
    if (std::string(list->getConnectionType(i)) != std::string("Ethernet")) {
      continue;
    }
    std::string serial = list->serialNumber(i);
    std::string ip_address = list->getIpAddress(i);
    RCLCPP_INFO(get_logger(), "Found Orbbec device: %s at %s", serial.c_str(),
                ip_address.c_str());
    serials_.push_back(serial);
    if (IsAlreadySpawned(serial)) continue;
    RCLCPP_INFO(get_logger(), "Spawning it...");

    const std::string node_name = std::string("orbbec_") + serial;
    rclcpp::NodeOptions node_options =
        rclcpp::NodeOptions()
            .append_parameter_override(
                rclcpp::Parameter("serial_number", serial))
            .append_parameter_override(
                rclcpp::Parameter("net_device_ip", ip_address));
    nodes_.push_back(std::make_unique<orbbec_camera::OBCameraNodeDriver>(
        node_name, "/", node_options));
  }
}

bool SpawnerNode::IsAlreadySpawned(const std::string& serial) const {
  for (const auto& node : nodes_) {
    std::string node_serial;
    if (!node->get_parameter<std::string>(std::string("serial_number"),
                                          node_serial)) {
      RCLCPP_ERROR(get_logger(), "Could not get serial number of a node");
      continue;
    }
    if (node_serial == serial) {
      return true;
    }
  }
  return false;
}

}  // namespace flowstate_orbbec

