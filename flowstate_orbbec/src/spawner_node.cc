#include "flowstate_orbbec/spawner_node.h"
#include "orbbec_camera/ob_camera_node_driver.h"

namespace flowstate_orbbec {

SpawnerNode::SpawnerNode()
    : flowstate_common::CameraSpawnerNode("orbbec_spawner", "orbbec", std::chrono::seconds(10)) {
  UpdateCameras();
}

void SpawnerNode::UpdateCameras() {
  // ob::Context::setLoggerSeverity(OBLogSeverity::OB_LOG_SEVERITY_OFF);
  auto context = std::make_unique<ob::Context>();
  auto list = context->queryDeviceList();
  
  std::vector<std::string> current_serials;
  for (size_t i = 0; i < list->deviceCount(); i++) {
    if (std::string(list->getConnectionType(i)) != std::string("Ethernet")) {
      continue;
    }
    std::string serial = list->serialNumber(i);
    std::string ip_address = list->getIpAddress(i);
    RCLCPP_INFO(get_logger(), "Found Orbbec device: %s at %s", serial.c_str(),
                ip_address.c_str());
    current_serials.push_back(serial);
    if (IsAlreadySpawned(serial)) continue;
    RCLCPP_INFO(get_logger(), "Spawning it...");

    spawned_nodes_.push_back(std::make_shared<AdapterNode>(serial, ip_address));
  }

  // See if any camera nodes have crashed. If so, close them so we can respawn
  CleanupExitedNodes();

  // Update the Base Class with the list of active serials
  SetDiscoveredSerials(current_serials);
}

bool SpawnerNode::IsAlreadySpawned(const std::string& serial) const {
  for (const auto& spawned_node : spawned_nodes_) {
    if (spawned_node->HasSerial(serial)) {
      return true;
    }
  }
  return false;
}

}  // namespace flowstate_orbbec