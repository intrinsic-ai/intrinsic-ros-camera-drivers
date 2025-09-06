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
        ob::Context::setLoggerSeverity(OBLogSeverity::OB_LOG_SEVERITY_OFF);
        auto context = std::make_unique<ob::Context>();
        auto list = context->queryDeviceList();
        for (size_t i = 0; i < list->deviceCount(); i++) {
          if (std::string(list->getConnectionType(i)) != std::string("Ethernet")) {
            continue;
          }
          std::string serial = list->serialNumber(i);
          RCLCPP_INFO(get_logger(), "Found Orbbec device: %s", serial.c_str());
          snapshot_interfaces::msg::DiscoveredCamera camera;
          camera.driver_type = "orbbec";
          camera.camera_id = serial;
          response->cameras.push_back(camera);
        }
        response->success = true;
      });
}

}  // namespace flowstate_orbbec
