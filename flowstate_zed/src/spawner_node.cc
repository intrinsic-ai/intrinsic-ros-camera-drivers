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

#include "flowstate_zed/spawner_node.h"

#include <algorithm>
#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "flowstate_zed/adapter_node.h"
#include "flowstate_zed/camera_discovery.h"
#include "rclcpp/rclcpp.hpp"
#include "sl/Camera.hpp"
#include "sl/CameraOne.hpp"
#include "snapshot_interfaces/msg/discovered_camera.hpp"
#include "snapshot_interfaces/srv/discover.hpp"

namespace flowstate_zed {
namespace {

constexpr std::chrono::milliseconds kSleepStep(100);
constexpr std::chrono::seconds kUpdatePeriod(10);
constexpr std::chrono::milliseconds kRespawnCooldown(2000);

// Sleeps for `duration` in small steps so SIGTERM ends waits promptly.
bool SleepWhileOk(std::chrono::milliseconds duration) {
  for (std::chrono::milliseconds elapsed(0); elapsed < duration;
       elapsed += kSleepStep) {
    if (!rclcpp::ok()) return false;
    std::this_thread::sleep_for(std::min(kSleepStep, duration - elapsed));
  }
  return rclcpp::ok();
}

CameraModel ToCameraModel(sl::MODEL model) {
  switch (model) {
    case sl::MODEL::ZED_X:
      return CameraModel::kZedX;
    case sl::MODEL::ZED_XM:
      return CameraModel::kZedXMini;
    case sl::MODEL::ZED_X_NANO:
      return CameraModel::kZedXNano;
    case sl::MODEL::ZED_XONE_GS:
      return CameraModel::kZedXOneGs;
    default:
      return CameraModel::kUnsupported;
  }
}

void AppendRawDevices(const std::vector<sl::DeviceProperties>& devices,
                      std::vector<RawDevice>& out) {
  for (const sl::DeviceProperties& device : devices) {
    out.push_back({
        .serial_number = device.serial_number,
        .model = ToCameraModel(device.camera_model),
        .available = device.camera_state == sl::CAMERA_STATE::AVAILABLE,
        .model_label = sl::toString(device.camera_model).c_str(),
    });
  }
}

std::vector<DiscoveredCamera> EnumerateCameras(const rclcpp::Logger& logger) {
  const auto enumerate_pass = [&logger](int attempt) -> DiscoveryPass {
    std::vector<RawDevice> raw_devices;
    AppendRawDevices(sl::Camera::getDeviceList(), raw_devices);
    if (!rclcpp::ok()) return {};
    AppendRawDevices(sl::CameraOne::getDeviceList(), raw_devices);
    if (!rclcpp::ok()) return {};
    const auto log_unsupported = [&logger, attempt](const RawDevice& device) {
      if (attempt == 0) {
        RCLCPP_WARN(logger, "Ignoring unsupported camera %u (%s)",
                    device.serial_number, device.model_label.c_str());
      }
    };
    return CollectSupportedDevices(raw_devices, log_unsupported);
  };

  const auto reboot_gmsl = [&logger]() {
    RCLCPP_WARN(logger,
                "GMSL camera unavailable or missing at startup; resetting "
                "Argus via ZEDX_Daemon");
    const sl::ERROR_CODE err = sl::Camera::reboot(sl::INPUT_TYPE::GMSL);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN(logger, "GMSL reset returned %s", sl::toString(err).c_str());
    }
  };

  std::vector<DiscoveredCamera> cameras = SettleDevices(
      enumerate_pass, reboot_gmsl, SleepWhileOk, []() { return rclcpp::ok(); });
  if (!rclcpp::ok()) return {};
  if (cameras.empty()) {
    RCLCPP_ERROR(logger, "No ZED X cameras found");
    return cameras;
  }
  for (const DiscoveredCamera& camera : cameras) {
    RCLCPP_INFO(logger, "Found %s camera %s", camera.driver_type.c_str(),
                camera.serial.c_str());
  }
  return cameras;
}

}  // namespace

SpawnerNode::SpawnerNode()
    : rclcpp::Node("zed_spawner"), cameras_(EnumerateCameras(get_logger())) {
  using snapshot_interfaces::srv::Discover;
  discover_service_ = create_service<Discover>(
      "/cameras/discover",
      [this](std::shared_ptr<rmw_request_id_t>,
             std::shared_ptr<Discover::Request>,
             std::shared_ptr<Discover::Response> response) {
        RCLCPP_INFO_ONCE(get_logger(),
                         "Discover service called (logging once)");
        for (const DiscoveredCamera& discovered : cameras_) {
          snapshot_interfaces::msg::DiscoveredCamera camera;
          camera.driver_type = discovered.driver_type;
          camera.camera_id = discovered.serial;
          response->cameras.push_back(std::move(camera));
        }
        response->success = true;
      });
  timer_ = create_wall_timer(kUpdatePeriod, [this]() { UpdateCameras(); });
}

SpawnerNode::~SpawnerNode() {
  timer_.reset();
  discover_service_.reset();
  std::vector<std::shared_ptr<AdapterNode>> nodes_to_destroy;
  {
    absl::MutexLock lock(&nodes_mutex_);
    nodes_to_destroy = std::move(spawned_nodes_);
  }
  nodes_to_destroy.clear();
  RCLCPP_INFO(get_logger(), "Camera SpawnerNode shutdown complete");
}

void SpawnerNode::UpdateCameras() {
  std::vector<std::shared_ptr<AdapterNode>> exited;
  {
    absl::MutexLock lock(&nodes_mutex_);
    for (auto it = spawned_nodes_.begin(); it != spawned_nodes_.end();) {
      if ((*it)->HasExitedThread()) {
        RCLCPP_INFO(get_logger(), "Camera %s thread exited. Removing node.",
                    (*it)->GetSerial().c_str());
        exited.push_back(std::move(*it));
        it = spawned_nodes_.erase(it);
      } else {
        ++it;
      }
    }
  }
  for (std::shared_ptr<AdapterNode>& node : exited) {
    node.reset();
    if (!SleepWhileOk(kRespawnCooldown)) return;
  }
  if (!rclcpp::ok()) return;

  std::vector<DiscoveredCamera> to_spawn;
  {
    absl::MutexLock lock(&nodes_mutex_);
    for (const DiscoveredCamera& camera : cameras_) {
      const bool already_spawned = std::ranges::any_of(
          spawned_nodes_, [&camera](const std::shared_ptr<AdapterNode>& node) {
            return node->GetSerial() == camera.serial;
          });
      if (!already_spawned) {
        to_spawn.push_back(camera);
      }
    }
  }

  for (const DiscoveredCamera& camera : to_spawn) {
    if (!rclcpp::ok()) return;
    RCLCPP_INFO(get_logger(), "Spawning adapter for %s camera %s",
                camera.driver_type.c_str(), camera.serial.c_str());
    try {
      std::shared_ptr<AdapterNode> node = std::make_shared<AdapterNode>(camera);
      absl::MutexLock lock(&nodes_mutex_);
      spawned_nodes_.push_back(std::move(node));
    } catch (const std::exception& e) {
      // For example when rclcpp is shut down meanwhile. The timer will retry
      // this camera on its next pass.
      RCLCPP_ERROR(get_logger(), "Could not create adapter for camera %s: %s",
                   camera.serial.c_str(), e.what());
    }
  }
}

}  // namespace flowstate_zed
