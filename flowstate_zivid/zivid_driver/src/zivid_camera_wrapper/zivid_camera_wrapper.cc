// Copyright 2024 Zivid AS
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Zivid AS nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*
 * Minimal Zivid Camera Wrapper Node
 * 
 * This node automatically detects all connected Zivid cameras and provides
 * information about them. It can be used to spawn individual ZividCamera 
 * node instances for each camera using the zivid_camera package.
 * 
 * Usage:
 *   ros2 run sdk_examples_ros_services zivid_camera_wrapper
 * 
 * Or with launch file:
 *   ros2 launch sdk_examples_ros_services zivid_camera_wrapper.launch.py
 */

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
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "zivid_camera_wrapper/zivid_camera_wrapper.h"
#include "zivid_camera_wrapper/zivid_camera_node.h"
#include "snapshot_interfaces/srv/discover.hpp"

using ::snapshot_interfaces::srv::Describe;
// using ::snapshot_interfaces::srv::Snapshot;
using ::snapshot_interfaces::srv::Discover;


namespace zivid_camera_wrapper
{

namespace ParamNames
{
constexpr auto serial_number = "serial_number";
constexpr auto file_camera_path = "file_camera_path";
constexpr auto frame_id = "frame_id";
constexpr auto color_space = "color_space";
constexpr auto intrinsics_source = "intrinsics_source";
}  // namespace ParamNames

absl::StatusOr<std::shared_ptr<ZividCameraWrapper>> ZividCameraWrapper::Create()
{
  try {
    auto wrapper = std::make_shared<ZividCameraWrapper>();
    
    return wrapper;
  } catch (const std::exception& e) {
    return absl::Status(absl::StatusCode::kInternal, 
                       absl::StrCat("Failed to create ZividCameraWrapper: ", e.what()));
  }
}

ZividCameraWrapper::ZividCameraWrapper(const rclcpp::NodeOptions & options)
  : rclcpp::Node("zivid_spawner", options) //("zivid_wrapper_node", options)
{
  RCLCPP_INFO(get_logger(), "Starting Zivid Camera Wrapper Node...");

  // Initialize Zivid Application
  zivid_app_ = std::make_unique<Zivid::Application>();

  const auto file_camera_path = declare_parameter(ParamNames::file_camera_path, "");
  refreshCameraList(file_camera_path); 

  // Create discovery service
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
        
        // response->discovered_cameras.clear();
        response->cameras.clear();

        for (const auto& camera : cameras_) {
          response->cameras.push_back(camera);
        }
        
        response->success = true;
        RCLCPP_INFO(get_logger(), "=== DISCOVERY SERVICE COMPLETE ===");
      });
  RCLCPP_INFO(get_logger(), "Zivid Camera Wrapper ready!");
  
  // Create individual ZividCamera nodes for each discovered camera
}

ZividCameraWrapper::~ZividCameraWrapper() {
  shutdownCameraNodes();
  if (timer_) {
    timer_->reset();
  }
  RCLCPP_INFO(get_logger(), "Zivid Camera Wrapper shutdown complete");
  zivid_app_.reset();
}

std::vector<std::string> ZividCameraWrapper::getCameraNodeNames() const
{
  absl::MutexLock lock(&cameras_mutex_);
  
  std::vector<std::string> node_names;
  for (const auto& camera : cameras_) {
    // The node name should match the one created in refreshCameraList
    node_names.push_back("zivid_" + camera.camera_id);
  }
  
  return node_names;
}

void ZividCameraWrapper::refreshCameraList(const std::string & file_camera_path)
{
  absl::MutexLock lock(&cameras_mutex_);
  
  // Clear existing cameras
  cameras_.clear();
  
  // Shutdown existing camera nodes before refreshing.
  // This also handles resetting the Zivid::Application instance.
  if (!camera_nodes_.empty()) {
    shutdownCameraNodes();
  }

  // Discover cameras
  std::vector<std::shared_ptr<Zivid::Camera>> cameras_discovered;
  
  // Check if file_camera_path is explicitly set and not empty
  if (!file_camera_path.empty()) {
    RCLCPP_INFO(get_logger(), "Using file camera path: %s", file_camera_path.c_str());
    cameras_discovered.push_back(std::make_shared<Zivid::Camera>(zivid_app_->createFileCamera(file_camera_path)));
  } else {
    RCLCPP_INFO(get_logger(), "Discovering physical cameras...");
    for (auto& cam : zivid_app_->cameras()) {
      cameras_discovered.push_back(std::make_shared<Zivid::Camera>(cam));
    }
    RCLCPP_INFO(get_logger(), "Found %zu physical camera(s)", cameras_discovered.size());
  }
  RCLCPP_INFO_STREAM(get_logger(), cameras_discovered.size() << " camera(s) found");

  for (size_t i = 0; i < cameras_discovered.size(); ++i) {
    const auto& camera_ptr = cameras_discovered[i];
    RCLCPP_INFO(get_logger(), "Camera %zu: Serial = %s", i,
                camera_ptr->info().serialNumber().toString().c_str());
  }

  for (const auto& camera : cameras_discovered) {
    // Create DiscoveredCamera message
    snapshot_interfaces::msg::DiscoveredCamera discovered_camera;
    discovered_camera.driver_type = "zivid";
    discovered_camera.camera_id = camera->info().serialNumber().toString();

    rclcpp::NodeOptions node_options;

    cameras_.push_back(discovered_camera);

    // Create unique node name based on discovered serial
    std::string node_name = discovered_camera.driver_type + "_" + discovered_camera.camera_id; //std::string("zivid_camera_") + discovered_camera.serial;

    // Set the serial number parameter for this specific camera
    std::vector<rclcpp::Parameter> parameters;
    parameters.emplace_back("serial_number", discovered_camera.camera_id);
    
    // For file cameras, also pass the file path to prevent duplication
    if (!file_camera_path.empty()) {
      parameters.emplace_back("file_camera_path", file_camera_path);
    }
    
    node_options.parameter_overrides(parameters);
    
    try {
      // Create ZividCamera node with the specific camera object (avoids duplication)
      auto camera_node = std::make_shared<zivid_camera_node::ZividCamNode>(node_name, node_options, *camera, *zivid_app_.get());
      camera_nodes_.push_back(camera_node);
        RCLCPP_INFO(get_logger(), "Created ZividCamera node for camera %s", discovered_camera.camera_id.c_str());
        
      // Create a thread to spin this camera node
      camera_threads_.emplace_back([camera_node]() {
        rclcpp::spin(camera_node);
      });
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to create ZividCamera node for camera "
                                          << discovered_camera.camera_id << ": " << e.what());
    }
  }

}

void ZividCameraWrapper::shutdownCameraNodes()
{
  if (camera_nodes_.empty() && camera_threads_.empty()) {
    return;
  }
  RCLCPP_INFO(get_logger(), "Shutting down %zu camera node(s)...", camera_nodes_.size());

  // Request nodes to shut down.
  for (auto& node : camera_nodes_) {
    // The node might be null if creation failed but was still added to the list.
    if (node) {
      rclcpp::shutdown(node->get_node_base_interface()->get_context());
    }
  }

  for (auto& thread : camera_threads_) {
    if (thread.joinable()) {
      thread.join();
    }
  }
  camera_nodes_.clear();
  camera_threads_.clear();

  if (zivid_app_) {
    RCLCPP_INFO(get_logger(), "Resetting Zivid Application to free resources...");
    zivid_app_.reset();
    zivid_app_ = std::make_unique<Zivid::Application>();
  }
}
}
