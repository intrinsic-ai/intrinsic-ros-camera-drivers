# Adding a New Camera Driver to Flowstate ROS

This guide explains how to create a new camera driver adapter for Flowstate using the common base classes provided in `flowstate_common`.

## Overview

The refactored architecture provides two base classes to reduce code duplication:

1. **`CameraAdapterNode`** (`flowstate_common`): Handles communication with a single camera
2. **`CameraSpawnerNode`** (`flowstate_common`): Discovers cameras and spawns adapter nodes

## Quick Start: Creating a New Camera Driver

### Step 1: Create the Package Structure

```bash
mkdir flowstate_<camera_name>
cd flowstate_<camera_name>
mkdir -p include/flowstate_<camera_name> src
touch CMakeLists.txt package.xml
```

### Step 2: Create the Adapter Node Header

File: `include/flowstate_<camera_name>/adapter_node.h`

```cpp
#ifndef FLOWSTATE_<CAMERA_NAME>_ADAPTER_NODE_H_
#define FLOWSTATE_<CAMERA_NAME>_ADAPTER_NODE_H_

#include <memory>
#include <string>

#include "flowstate_common/camera_adapter_node.h"
#include "<vendor>/<camera_driver>.h"  // Your camera driver header

namespace flowstate_<camera_name> {

class AdapterNode : public flowstate_common::CameraAdapterNode {
 public:
  AdapterNode(const std::string& serial, const std::string& ip_address);

 private:
  absl::Status Main() override;
  std::string ColorImageTopic() const override;

  std::unique_ptr<<VendorDriver>> camera_driver_;
};

}  // namespace flowstate_<camera_name>

#endif
```

### Step 3: Implement the Adapter Node

File: `src/adapter_node.cc`

```cpp
#include "flowstate_<camera_name>/adapter_node.h"

#include <memory>
#include "absl/strings/str_format.h"

namespace flowstate_<camera_name> {

AdapterNode::AdapterNode(const std::string& serial,
                         const std::string& ip_address)
    : flowstate_common::CameraAdapterNode(serial, ip_address, "<camera_name>") {
  // Initialize your camera driver
  rclcpp::NodeOptions driver_options = rclcpp::NodeOptions()
      .append_parameter_override(rclcpp::Parameter("serial", serial));
  
  camera_driver_ = std::make_unique<VendorDriver>(driver_options);

  // Subscribe to camera topics
  color_info_sub_ = SubscribeToCameraInfo(
      absl::StrFormat("/<camera_name>/camera_%s/camera_info", serial_),
      [this](sensor_msgs::msg::CameraInfo::UniquePtr msg) {
        absl::MutexLock lock(&this->camera_info_mutex_);
        this->color_camera_info_ = std::move(msg);
      });

  color_image_sub_ = SubscribeToImage(
      ColorImageTopic(),
      [this](sensor_msgs::msg::Image::UniquePtr msg) {
        {
          absl::MutexLock timeout_lock(&this->timeout_mutex_);
          this->t_last_color_image_ = this->get_clock()->now();
        }
        absl::MutexLock lock(&this->image_mutex_);
        this->color_image_ = std::move(msg);
      });

  // Create Flowstate services (describe and snapshot)
  CreateFlowstateServices();

  // Start the background executor thread
  StartExecutorThread();
}

std::string AdapterNode::ColorImageTopic() const {
  return absl::StrFormat("/<camera_name>/camera_%s/image_raw", serial_);
}

absl::Status AdapterNode::Main() {
  RCLCPP_INFO(get_logger(), "AdapterNode::Main()");
  t_last_color_image_ = get_clock()->now();
  rclcpp::executors::SingleThreadedExecutor executor;

  liveness_timer_ = create_wall_timer(std::chrono::seconds(1),
      [this, &executor]() {
        absl::MutexLock timeout_lock(&timeout_mutex_);
        if ((get_clock()->now() - t_last_color_image_).seconds() > 30.0) {
          RCLCPP_ERROR(get_logger(), "No new image arrived for 30 seconds");
          executor.cancel();
        }
      });

  executor.add_node(this->get_node_base_interface());
  executor.add_node(camera_driver_->get_node_base_interface());
  executor.spin();
  return absl::OkStatus();
}

}  // namespace flowstate_<camera_name>
```

### Step 4: Create the Spawner Node

File: `include/flowstate_<camera_name>/spawner_node.h`

```cpp
#ifndef FLOWSTATE_<CAMERA_NAME>_SPAWNER_NODE_H_
#define FLOWSTATE_<CAMERA_NAME>_SPAWNER_NODE_H_

#include <memory>
#include <string>
#include <vector>

#include "flowstate_common/camera_spawner_node.h"
#include "flowstate_<camera_name>/adapter_node.h"

namespace flowstate_<camera_name> {

class SpawnerNode : public flowstate_common::CameraSpawnerNode {
 public:
  SpawnerNode();

 private:
  void UpdateCameraList() override;
  bool IsAlreadySpawned(const std::string& serial) const override;
  std::string GetDiscoveredCameraSerial(size_t index) const override;

  std::vector<std::unique_ptr<AdapterNode>> spawned_nodes_;
};

}  // namespace flowstate_<camera_name>

#endif
```

File: `src/spawner_node.cc`

```cpp
#include "flowstate_<camera_name>/spawner_node.h"

#include <memory>
#include "rclcpp/rclcpp.hpp"

namespace flowstate_<camera_name> {

SpawnerNode::SpawnerNode()
    : flowstate_common::CameraSpawnerNode("<camera_name>_spawner", 10.0) {
  // Initial discovery happens automatically in base class constructor
}

void SpawnerNode::UpdateCameraList() {
  // Query your camera SDK for connected devices
  auto devices = YourCameraSDK::discoverDevices();

  for (const auto& device : devices) {
    std::string serial = device.getSerialNumber();
    std::string model = device.getModelName();
    std::string ip = device.getIpAddress();

    RCLCPP_INFO(get_logger(), "Found %s device: %s at %s",
                model.c_str(), serial.c_str(), ip.c_str());

    RegisterDiscoveredCamera(serial, model, ip);

    if (IsAlreadySpawned(serial)) continue;

    RCLCPP_INFO(get_logger(), "Spawning adapter for %s", serial.c_str());
    spawned_nodes_.push_back(
        std::make_unique<AdapterNode>(serial, ip));
  }

  // Clean up crashed nodes
  for (auto it = spawned_nodes_.begin(); it != spawned_nodes_.end();) {
    if ((*it)->HasExitedThread()) {
      RCLCPP_INFO(get_logger(), "Camera %s exited, removing",
                  (*it)->GetSerial().c_str());
      it = spawned_nodes_.erase(it);
    } else {
      ++it;
    }
  }
}

bool SpawnerNode::IsAlreadySpawned(const std::string& serial) const {
  for (const auto& node : spawned_nodes_) {
    if (node->HasSerial(serial)) return true;
  }
  return false;
}

std::string SpawnerNode::GetDiscoveredCameraSerial(size_t index) const {
  auto devices = YourCameraSDK::discoverDevices();
  if (index < devices.size()) {
    return devices[index].getSerialNumber();
  }
  return "";
}

}  // namespace flowstate_<camera_name>
```

### Step 5: Create CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(flowstate_<camera_name>)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-g -Wall -Wextra -Wpedantic)

find_package(abseil_cpp_vendor REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(flowstate_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(snapshot_interfaces REQUIRED)
find_package(<vendor>_<camera_sdk> REQUIRED)

include_directories(include)

add_library(flowstate_<camera_name>
  src/adapter_node.cc
  src/spawner_node.cc
)
target_link_libraries(flowstate_<camera_name>
  flowstate_common
  absl::status
  absl::synchronization
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${snapshot_interfaces_TARGETS}
)
ament_target_dependencies(flowstate_<camera_name>
  flowstate_common
  <vendor>_<camera_sdk>
)

add_executable(spawner_node src/ros_main.cc)
target_link_libraries(spawner_node flowstate_<camera_name>)

install(TARGETS spawner_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS flowstate_<camera_name> DESTINATION lib)

ament_package()
```

### Step 6: Create package.xml

```xml
<?xml version="1.0"?>
<package format="2">
  <name>flowstate_<camera_name></name>
  <version>0.0.1</version>
  <description>Flowstate adapter for <Camera Name> cameras</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>absl_cpp</depend>
  <depend>flowstate_common</depend>
  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>snapshot_interfaces</depend>
  <depend><vendor>_<camera_sdk></depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## Advanced: Customizing Behavior

### Multiple Image Types

To support multiple image types (color, IR, depth), override `BuildDescribeResponse()` and `BuildSnapshotResponse()`:

```cpp
bool AdapterNode::BuildDescribeResponse(
    snapshot_interfaces::srv::Describe::Response& response) override {
  // Add color sensor
  snapshot_interfaces::msg::SensorInfo color_info;
  color_info.sensor_name = "color";
  color_info.topic_name = ColorImageTopic();
  color_info.sensor_type = snapshot_interfaces::msg::SensorInfo::IMAGE;
  response.sensors.push_back(color_info);

  // Add IR sensor
  snapshot_interfaces::msg::SensorInfo ir_info;
  ir_info.sensor_name = "ir";
  ir_info.topic_name = IrImageTopic();
  ir_info.sensor_type = snapshot_interfaces::msg::SensorInfo::IMAGE;
  response.sensors.push_back(ir_info);

  return true;
}
```

### Camera Parameters

To support runtime parameter configuration:

1. Declare parameters in the constructor
2. Register parameter callbacks
3. Handle parameter changes by calling your camera SDK APIs

See `flowstate_orbbec` for a complete example with exposure, gain, and white balance controls.

### Custom Image Processing

If your camera outputs BGR data, convert it to RGB:

```cpp
#include "flowstate_common/image_utils.h"

// In BuildSnapshotResponse:
auto rgb_image = flowstate_common::ConvertBgrImageToRgb(*color_image_);
```

## Testing Your Camera Driver

1. **Build the package:**
   ```bash
   cd ~/ros_ws
   colcon build --packages-up-to flowstate_<camera_name>
   ```

2. **Run the spawner node:**
   ```bash
   source install/setup.bash
   ros2 run flowstate_<camera_name> spawner_node
   ```

3. **Call the discover service:**
   ```bash
   ros2 service call /cameras/discover snapshot_interfaces/srv/Discover
   ```

4. **Call snapshot:**
   ```bash
   ros2 service call /<camera_name>_<serial>/snapshot snapshot_interfaces/srv/Snapshot
   ```

## Common Patterns

### Pattern 1: USB-based cameras (no IP address)
- Don't use the `ip_address` parameter
- Return empty string from `GetDiscoveredCameraIp()`

### Pattern 2: Network cameras with authentication
- Handle credentials in your discovery SDK
- Pass configuration through `rclcpp::NodeOptions`

### Pattern 3: Cameras with multiple sensor streams
- Subscribe to each stream's camera_info and image topics
- Implement `BuildDescribeResponse()` and `BuildSnapshotResponse()` to include all streams
- Use separate mutexes for each stream's data

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Adapter node crashes | Check timeout in `Main()` - ensure images arrive within 30 seconds |
| Services not responding | Verify `CreateFlowstateServices()` is called in constructor |
| Image data not received | Verify topic names match your camera driver's output |
| Parameter changes not taking effect | Ensure parameter callbacks are registered in `InitializeParameters()` |

## See Also

- **Luxonis Example**: `flowstate_luxonis/` - Simple USB camera with color output only
- **Orbbec Example**: `flowstate_orbbec/` - Network camera with color + IR + depth + parameters
- **Zivid Example**: `flowstate_zivid/` - Complex camera with custom capture logic

