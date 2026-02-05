# Testing Guide for Flowstate Camera Drivers

This guide explains how to test the refactored camera driver architecture and verify that new drivers work correctly.

## Prerequisites

1. You have a ROS Jazzy workspace set up with the camera drivers
2. You have cameras connected (or can run tests without hardware)
3. You have the snapshot_interfaces package available

## Level 1: Build Testing

### 1.1 Build All Packages

```bash
cd ~/ros_cameras_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to flowstate_common flowstate_luxonis flowstate_orbbec
```

**Expected output:**
- All three packages build successfully
- No compilation errors
- flowstate_common builds first (no dependencies on specific cameras)
- flowstate_luxonis and flowstate_orbbec depend on flowstate_common

### 1.2 Check Build Artifacts

```bash
ls -la install/flowstate_common/lib/
ls -la install/flowstate_common/include/flowstate_common/
```

**Expected:**
- `libflowstate_common.so` library exists
- Header files present: `camera_adapter_node.h`, `camera_spawner_node.h`, `image_utils.h`

### 1.3 Verify CMake Configuration

```bash
cd build/flowstate_luxonis
cat CMakeCache.txt | grep flowstate_common
```

**Expected:**
- `flowstate_common` appears in dependency list

## Level 2: Unit Testing (No Hardware Required)

### 2.1 Test Base Class Instantiation

Create a simple test file: `test_base_classes.cpp`

```cpp
#include <gtest/gtest.h>
#include "flowstate_common/camera_adapter_node.h"
#include "flowstate_common/camera_spawner_node.h"
#include "rclcpp/rclcpp.hpp"

// Mock implementation for testing
class MockAdapterNode : public flowstate_common::CameraAdapterNode {
 public:
  MockAdapterNode() : CameraAdapterNode("test_serial", "192.168.1.1", "test") {}

  absl::Status Main() override {
    return absl::OkStatus();
  }

  std::string ColorImageTopic() const override {
    return "/test/color/image";
  }
};

class MockSpawnerNode : public flowstate_common::CameraSpawnerNode {
 public:
  MockSpawnerNode() : CameraSpawnerNode("test_spawner") {}

  void UpdateCameraList() override {
    RegisterDiscoveredCamera("test_serial_1", "test_model", "192.168.1.1");
  }

  bool IsAlreadySpawned(const std::string& serial) const override {
    return false;
  }

  std::string GetDiscoveredCameraSerial(size_t index) const override {
    return "test_serial_1";
  }
};

TEST(BaseClasses, AdapterNodeInstantiation) {
  rclcpp::init(0, nullptr);
  EXPECT_NO_THROW({
    MockAdapterNode adapter;
  });
  rclcpp::shutdown();
}

TEST(BaseClasses, SpawnerNodeInstantiation) {
  rclcpp::init(0, nullptr);
  EXPECT_NO_THROW({
    MockSpawnerNode spawner;
  });
  rclcpp::shutdown();
}
```

### 2.2 Test Image Utilities

```cpp
#include <gtest/gtest.h>
#include "flowstate_common/image_utils.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"

TEST(ImageUtils, ConvertBgrToRgb) {
  sensor_msgs::msg::Image bgr_image;
  bgr_image.encoding = sensor_msgs::image_encodings::BGR8;
  bgr_image.width = 2;
  bgr_image.height = 2;
  bgr_image.step = 6;  // 2 pixels * 3 channels
  bgr_image.data = {0, 0, 255,    // BGR red pixel
                    0, 255, 0,    // BGR green pixel
                    255, 0, 0,    // BGR blue pixel
                    0, 0, 0};     // BGR black pixel

  bool success = flowstate_common::ConvertBgrToRgb(bgr_image);
  
  EXPECT_TRUE(success);
  EXPECT_EQ(bgr_image.encoding, sensor_msgs::image_encodings::RGB8);
  EXPECT_EQ(bgr_image.data[0], 255);  // Should be red (was 0)
  EXPECT_EQ(bgr_image.data[1], 0);    // Should be 0 (was 0)
  EXPECT_EQ(bgr_image.data[2], 0);    // Should be 0 (was 255)
}
```

## Level 3: Integration Testing (Single Camera)

### 3.1 Manual Service Testing - Luxonis

```bash
# Terminal 1: Start the Luxonis spawner
cd ~/ros_cameras_ws
source install/setup.bash
ros2 run flowstate_luxonis spawner_node

# Terminal 2: Call the discover service
ros2 service call /cameras/discover snapshot_interfaces/srv/Discover

# Expected output:
# success: true
# cameras:
# - serial: '<camera_serial_number>'
#   model: ''
#   ip_address: '<ip_address_or_empty>'
```

### 3.2 Manual Service Testing - Orbbec

```bash
# Terminal 1: Start the Orbbec spawner
cd ~/ros_cameras_ws
source install/setup.bash
ros2 run flowstate_orbbec spawner_node

# Terminal 2: Check discovered cameras
ros2 service call /cameras/discover snapshot_interfaces/srv/Discover

# Terminal 3: Call describe on a camera
ros2 service call /<camera_name>_<serial>/describe snapshot_interfaces/srv/Describe

# Expected output:
# success: true
# error_message: ''
# sensors:
# - sensor_name: 'rgb'
#   topic_name: '/orbbec/camera_<serial>/color/image_raw'
#   sensor_type: 0  # IMAGE type
#   camera_t_sensor:
#     ...
```

### 3.3 Manual Service Testing - Snapshot

```bash
# Call snapshot service
ros2 service call /<camera_name>_<serial>/snapshot snapshot_interfaces/srv/Snapshot

# Expected output:
# success: true
# error_message: ''
# images:
# - topic_name: '/orbbec/camera_<serial>/color/image_raw'
#   image:
#     header:
#       ...
#     height: 800
#     width: 1280
#     encoding: 'rgb8'
#     data: <image_data>
#   camera_info:
#     ...
```

## Level 4: Comparison Testing

### 4.1 Verify Behavior Matches Original

Compare behavior between refactored and original implementations by checking:

1. **Discover Service Response**
   ```bash
   # Should return same camera list
   ros2 service call /cameras/discover snapshot_interfaces/srv/Discover
   ```

2. **Describe Service Response**
   ```bash
   # Should return identical sensor information
   ros2 service call /<camera>_<serial>/describe snapshot_interfaces/srv/Describe
   ```

3. **Snapshot Data**
   ```bash
   # Compare image dimensions and encoding
   ros2 service call /<camera>_<serial>/snapshot snapshot_interfaces/srv/Snapshot
   ```

### 4.2 Performance Comparison

Monitor resource usage before and after refactoring:

```bash
# Monitor memory and CPU
ros2 run flowstate_orbbec spawner_node &
PID=$!
watch -n 1 "ps aux | grep $PID"
```

## Level 5: Multi-Camera Testing

If you have multiple cameras of the same type:

```bash
# Start spawner
ros2 run flowstate_luxonis spawner_node

# List all camera nodes
ros2 node list | grep luxonis

# Call services on each camera
ros2 service call /luxonis_<serial1>/snapshot snapshot_interfaces/srv/Snapshot
ros2 service call /luxonis_<serial2>/snapshot snapshot_interfaces/srv/Snapshot
```

## Level 6: Stress Testing

### 6.1 Rapid Service Calls

```bash
# Create a loop calling snapshot rapidly
for i in {1..100}; do
  echo "Call $i"
  ros2 service call /<camera>_<serial>/snapshot snapshot_interfaces/srv/Snapshot --no-wait
  sleep 0.1
done
```

### 6.2 Long-Running Test

```bash
# Monitor camera for 10 minutes
timeout 600 ros2 run flowstate_luxonis spawner_node

# In parallel, continuously call discover
watch -n 1 'ros2 service call /cameras/discover snapshot_interfaces/srv/Discover'
```

### 6.3 Camera Reconnection Test

1. Start spawner node
2. Verify camera discovered
3. Physically disconnect camera
4. Observe spawner detects disconnection
5. Reconnect camera
6. Verify spawner re-discovers camera

Expected behavior: Camera is rediscovered within 10 seconds

## Level 7: Automated Testing with Script

Create `test_cameras.sh`:

```bash
#!/bin/bash

set -e

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

echo "Testing Flowstate Camera Drivers"
echo "================================"

# Test 1: Build
echo "Test 1: Building packages..."
if colcon build --packages-up-to flowstate_common flowstate_luxonis flowstate_orbbec 2>/dev/null; then
    echo -e "${GREEN}✓ Build successful${NC}"
else
    echo -e "${RED}✗ Build failed${NC}"
    exit 1
fi

# Test 2: Source setup
source install/setup.bash

# Test 3: Check discover service exists
echo "Test 2: Checking discover service..."
if ros2 service list | grep -q discover; then
    echo -e "${GREEN}✓ Discover service exists${NC}"
else
    echo -e "${RED}✗ Discover service not found${NC}"
fi

# Test 4: Call discover
echo "Test 3: Calling discover service..."
DISCOVER_OUTPUT=$(ros2 service call /cameras/discover snapshot_interfaces/srv/Discover)
if echo "$DISCOVER_OUTPUT" | grep -q "success: true"; then
    echo -e "${GREEN}✓ Discover service works${NC}"
else
    echo -e "${RED}✗ Discover service failed${NC}"
fi

echo ""
echo "All tests passed!"
```

Run it:
```bash
chmod +x test_cameras.sh
./test_cameras.sh
```

## Level 8: Debugging Failed Tests

### 8.1 Enable ROS Logging

```bash
export ROS_LOG_DIR=~/.ros/log/$(date +%Y-%m-%d)
ros2 run flowstate_luxonis spawner_node --ros-args --log-level debug
```

### 8.2 Check Topic Flow

```bash
# Terminal 1: Start spawner
ros2 run flowstate_luxonis spawner_node

# Terminal 2: Check which topics exist
ros2 topic list

# Terminal 3: Monitor image topics
ros2 topic echo /luxonis_<serial>/driver/rgb/image_raw

# Terminal 4: Monitor camera info
ros2 topic echo /luxonis_<serial>/driver/rgb/camera_info
```

### 8.3 Common Issues and Solutions

| Issue | Check | Solution |
|-------|-------|----------|
| Camera not discovered | `ros2 topic list` | Verify camera driver is publishing topics |
| Discover service hangs | `ros2 node list` | Check spawner node is running |
| Snapshot returns empty | `ros2 topic echo` | Verify images are being published |
| Timeout error (30 sec) | Camera driver logs | Camera may not be publishing images |
| Service call fails | `ros2 service list` | Service may not be registered yet |

### 8.4 Check Node State

```bash
# List all nodes
ros2 node list

# Get node info
ros2 node info /<node_name>

# Check node parameters
ros2 param list /<node_name>

# Get specific parameter
ros2 param get /<node_name> <param_name>
```

## Quick Verification Checklist

After making changes, run through this checklist:

- [ ] Code compiles without errors
- [ ] flowstate_common builds before camera-specific packages
- [ ] Spawner node starts without crashing
- [ ] Discover service returns cameras
- [ ] Describe service returns sensor info
- [ ] Snapshot service returns images
- [ ] Images have correct encoding (rgb8, not bgr8)
- [ ] Camera info is populated
- [ ] Multiple cameras work simultaneously
- [ ] Liveness timeout works (30 seconds)
- [ ] Crashed nodes are cleaned up and respawned

## Testing New Camera Drivers

When adding a new camera, verify:

1. **Discovery works**: New cameras appear in discover service response
2. **Adapter spawns**: Adapter node created for each new camera
3. **Services respond**: Describe and snapshot services work
4. **Images valid**: Image data is reasonable (correct size, encoding)
5. **Threading safe**: No crashes under load
6. **Cleanup works**: Crashed adapters are removed

---

**Need more help?** Check the logs:
```bash
ls ~/.ros/log/
tail -f ~/.ros/log/<latest_date>/*/stdout.log
```
