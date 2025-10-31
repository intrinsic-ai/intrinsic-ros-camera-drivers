#include "flowstate_zivid/spawner_node.h"

#include <Zivid/Application.h>
#include <Zivid/Camera.h>
#include <Zivid/CaptureAssistant.h>
#include <Zivid/Exception.h>
#include <Zivid/Experimental/Calibration.h>
#include <Zivid/Experimental/PointCloudExport.h>
#include <Zivid/Firmware.h>
#include <Zivid/Frame2D.h>
#include <Zivid/Image.h>
#include <Zivid/Settings2D.h>
#include <Zivid/Version.h>

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <thread>
#include <vector>

#include "flowstate_zivid/adapter_node.h"
#include "snapshot_interfaces/srv/discover.hpp"
#include "zivid_camera/zivid_camera.hpp"

using ::snapshot_interfaces::srv::Describe;
using ::snapshot_interfaces::srv::Discover;

namespace flowstate_zivid {

namespace ParamNames {
constexpr auto serial_number = "serial_number";
constexpr auto file_camera_path = "file_camera_path";
constexpr auto frame_id = "frame_id";
constexpr auto color_space = "color_space";
constexpr auto intrinsics_source = "intrinsics_source";
}  // namespace ParamNames

absl::StatusOr<std::shared_ptr<SpawnerNode>> SpawnerNode::Create() {
  try {
    auto spawner = std::make_shared<SpawnerNode>();

    return spawner;
  } catch (const std::exception& e) {
    return absl::Status(
        absl::StatusCode::kInternal,
        absl::StrCat(
            "Failed to create SpawnerNode for flowstate Zivid camera: ",
            e.what()));
  }
}

SpawnerNode::SpawnerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("zivid_spawner", options) {
  RCLCPP_INFO(get_logger(), "Starting Zivid SpawnerNode...");

  zivid_app_ = std::make_shared<Zivid::Application>();

  const auto file_camera_path =
      declare_parameter(ParamNames::file_camera_path, "");
  refreshCameraList(file_camera_path);

  using namespace std::placeholders;
  discover_service_ = create_service<Discover>(
      "/cameras/discover",
      [this](const std::shared_ptr<rmw_request_id_t>,
             const std::shared_ptr<Discover::Request>,
             const std::shared_ptr<Discover::Response> response) {
        RCLCPP_INFO(get_logger(), "=== DISCOVERY SERVICE CALLED ===");

        absl::MutexLock lock(&cameras_mutex_);

        RCLCPP_INFO(get_logger(), "Returning %d already discovered cameras",
                    static_cast<int>(cameras_.size()));

        response->cameras.clear();

        for (const auto& camera : cameras_) {
          response->cameras.push_back(camera);
        }

        response->success = true;
        RCLCPP_INFO(get_logger(), "=== DISCOVERY SERVICE COMPLETE ===");
      });
  RCLCPP_INFO(get_logger(), "Zivid SpawnerNode is ready!");
}

SpawnerNode::~SpawnerNode() {
  shutdownCameraNodes();
  if (timer_) {
    timer_->reset();
  }
  RCLCPP_INFO(get_logger(), "Zivid SpawnerNode shutdown complete");
  zivid_app_.reset();
}

std::vector<std::string> SpawnerNode::getCameraNodeNames() const {
  absl::MutexLock lock(&cameras_mutex_);

  std::vector<std::string> node_names;
  for (const auto& camera : cameras_) {
    node_names.push_back("zivid_" + camera.camera_id);
  }

  return node_names;
}

void SpawnerNode::refreshCameraList(const std::string& file_camera_path) {
  absl::MutexLock lock(&cameras_mutex_);

  cameras_.clear();
  cameras_discovered_.clear();

  if (!spawned_nodes_.empty()) {
    shutdownCameraNodes();
  }

  if (!file_camera_path.empty()) {
    RCLCPP_INFO(get_logger(), "Using file camera path: %s",
                file_camera_path.c_str());
    cameras_discovered_.push_back(std::make_shared<Zivid::Camera>(
        zivid_app_->createFileCamera(file_camera_path)));
  } else {
    RCLCPP_INFO(get_logger(), "Discovering physical cameras...");
    for (auto& cam : zivid_app_->cameras()) {
      cameras_discovered_.push_back(std::make_shared<Zivid::Camera>(cam));
    }
    RCLCPP_INFO(get_logger(), "Found %zu physical camera(s)",
                cameras_discovered_.size());
  }
  RCLCPP_INFO_STREAM(get_logger(), cameras_discovered_.size()
                                       << " camera(s) found");

  for (size_t i = 0; i < cameras_discovered_.size(); ++i) {
    const auto& camera_ptr = cameras_discovered_[i];
    RCLCPP_INFO(get_logger(), "Camera %zu: Serial = %s", i,
                camera_ptr->info().serialNumber().toString().c_str());
  }

  for (const auto& camera : cameras_discovered_) {
    snapshot_interfaces::msg::DiscoveredCamera discovered_camera;
    discovered_camera.driver_type = "zivid";
    discovered_camera.camera_id = camera->info().serialNumber().toString();

    rclcpp::NodeOptions node_options;

    cameras_.push_back(discovered_camera);

    // Set the serial number parameter for this specific camera
    std::vector<rclcpp::Parameter> parameters;
    parameters.emplace_back("serial_number", discovered_camera.camera_id);

    // For file cameras, also pass the file path to prevent duplication
    if (!file_camera_path.empty()) {
      parameters.emplace_back("file_camera_path", file_camera_path);
    }

    node_options.parameter_overrides(parameters);

    try {
      // Create AdapterNode node with the specific camera object (avoids
      // duplication)
      auto camera_node = std::make_shared<flowstate_zivid::AdapterNode>(
          discovered_camera.camera_id, node_options, camera, zivid_app_);
      spawned_nodes_.push_back(camera_node);
      RCLCPP_INFO(get_logger(), "Created AdapterNode for zivid camera %s",
                  discovered_camera.camera_id.c_str());

    } catch (const std::exception& e) {
      RCLCPP_ERROR_STREAM(
          get_logger(), "Failed to create AdapterNode for zivid camera "
                            << discovered_camera.camera_id << ": " << e.what());
    }
  }
}

void SpawnerNode::shutdownCameraNodes() {
  if (spawned_nodes_.empty() && camera_threads_.empty()) {
    return;
  }
  RCLCPP_INFO(get_logger(), "Shutting down %zu camera node(s)...",
              spawned_nodes_.size());

  // Request nodes to shut down.
  for (auto& node : spawned_nodes_) {
    // The node might be null if creation failed but was still added to the
    // list.
    if (node) {
      rclcpp::shutdown(node->get_node_base_interface()->get_context());
    }
  }

  for (auto& thread : camera_threads_) {
    if (thread.joinable()) {
      thread.join();
    }
  }
  spawned_nodes_.clear();
  camera_threads_.clear();

  if (zivid_app_) {
    RCLCPP_INFO(get_logger(),
                "Resetting Zivid Application to free resources...");
    zivid_app_.reset();
    zivid_app_ = std::make_shared<Zivid::Application>();
  }
}
}  // namespace flowstate_zivid
