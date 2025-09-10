#include "flowstate_orbbec/spawner_node.h"

#include <memory>

#include "orbbec_camera/ob_camera_node_driver.h"
#include "rclcpp/rclcpp.hpp"
#include "snapshot_interfaces/msg/discovered_camera.hpp"
#include "snapshot_interfaces/srv/discover.hpp"

namespace flowstate_orbbec {

using snapshot_interfaces::srv::Discover;

SpawnerNode::SpawnerNode()
    : Node(std::string("orbbec_spawner")) {
  discover_service_ = create_service<Discover>(
      std::string("/cameras/discover"),
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
  UpdateCameras();
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

    spawned_nodes_.push_back(std::make_unique<AdapterNode>(serial, ip_address));
  }

  // See if any camera nodes have crashed. If so, close them so we can respawn
  for (auto node_it = spawned_nodes_.begin();
       node_it != spawned_nodes_.end();) {
    if ((*node_it)->exited_thread_) {
      RCLCPP_INFO(get_logger(), "Camera %s has exited. Removing it.",
                  (*node_it)->serial_.c_str());
      node_it = spawned_nodes_.erase(node_it);
    } else {
      ++node_it;
    }
  }
}

bool SpawnerNode::IsAlreadySpawned(const std::string& serial) const {
  for (const auto& spawned_node : spawned_nodes_) {
    if (spawned_node->serial_ == serial) {
      return true;
    }
  }
  return false;
}

AdapterNode::AdapterNode(const std::string& serial,
                         const std::string& ip_address)
    : Node(std::string("orbbec_") + serial),
      serial_(serial),
      ip_address_(ip_address) {
  const std::string orbbec_node_name = std::string("orbbec_camera_node");
  const std::string orbbec_ns = std::string("orbbec/camera_") + serial;
  rclcpp::NodeOptions orbbec_node_options =
      rclcpp::NodeOptions()
          .append_parameter_override(rclcpp::Parameter("serial_number", serial))
          .append_parameter_override(
              rclcpp::Parameter("enumerate_net_device", true))
          .append_parameter_override(
              rclcpp::Parameter("enable_depth", false))
          .append_parameter_override(
              rclcpp::Parameter("enable_color", true))
          .append_parameter_override(
              rclcpp::Parameter("enable_ir", true));
  //            .append_parameter_override(
  //                rclcpp::Parameter("net_device_ip", ip_address))
  //            .append_parameter_override(
  //                rclcpp::Parameter("net_device_port", 8090));
  orbbec_node_ = std::make_unique<orbbec_camera::OBCameraNodeDriver>(
      orbbec_node_name, orbbec_ns, orbbec_node_options);
  thread_ = std::thread([this]() {
    const absl::Status status = this->main();
    if (!status.ok()) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "node thread error: " << status);
    } else {
      RCLCPP_INFO(this->get_logger(), "exited thread");
    }
    RCLCPP_INFO(this->get_logger(), "Destroying orbbec_camera_node...");
    orbbec_node_.reset();
    rclcpp::sleep_for(std::chrono::milliseconds(500));  // maybe this helps?
    RCLCPP_INFO(this->get_logger(), "Done destroying orbbec_camera_node");
    exited_thread_ = true;
  });
}

absl::Status AdapterNode::main() {
  RCLCPP_INFO(get_logger(), "AdapterNode::main()");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(this->get_node_base_interface());
  executor.add_node(orbbec_node_->get_node_base_interface());
  // TODO: add some other test for camera health, to exit this loop if it's bad
  while (rclcpp::ok()) {
    executor.spin_some();
    // maybe do something
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  return absl::OkStatus();
}

}  // namespace flowstate_orbbec

