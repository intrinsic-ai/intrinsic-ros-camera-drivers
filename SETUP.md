# Setup Guide for Flowstate Camera Drivers

This repository contains ROS 2 camera driver packages that have been refactored to use abstract base classes and shared utilities.

## Prerequisites

### Required
- **ROS 2 Jazzy** - The entire codebase targets ROS 2 Jazzy
- **colcon** - Build tool for ROS 2 workspaces
- **Python 3.9+** - Required by ROS 2
- **CMake 3.22+** - For building C++ packages
- **C++20 compiler** - GCC 11+ or Clang 13+

### Optional (depends on which camera drivers you use)
- **depthai-ros** - For Luxonis OAK camera support
- **OrbbecSDK** - For Orbbec camera support
- **Zivid SDK** - For Zivid camera support (not yet refactored)

## Installation Options

### Option 1: Docker (Recommended for quick testing)

A Docker environment with ROS 2 Jazzy pre-installed is the easiest way to test:

```bash
# Build a Docker image with ROS 2 Jazzy
docker run -it osrf/ros:jazzy-desktop bash

# Inside the container:
cd /workspace
git clone <repo-url> flowstate-ros-camera-drivers
cd flowstate-ros-camera-drivers
colcon build
```

### Option 2: Native Linux Installation

**Ubuntu 24.04 (Recommended for Jazzy)**

1. **Install ROS 2 Jazzy**
   ```bash
   # Add ROS 2 apt repository
   sudo curl -sSL https://repo.ros2.org/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://repo.ros2.org/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   
   sudo apt update
   sudo apt install ros-jazzy-desktop python3-colcon-common-extensions build-essential
   ```

2. **Source ROS 2 setup**
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

3. **Build the workspace**
   ```bash
   cd flowstate-ros-camera-drivers
   colcon build
   source install/setup.bash
   ```

**Ubuntu 22.04 (Humble - May need adjustments)**

ROS 2 Jazzy prefers Ubuntu 24.04. If using 22.04, install ROS 2 Humble and adapt the code:

```bash
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

Then modify `CMakeLists.txt` files to use `ament_cmake` from Humble instead.

### Option 3: WSL2 on Windows

1. **Install WSL2 with Ubuntu 24.04**
   ```powershell
   wsl --install Ubuntu-24.04
   ```

2. **Inside WSL2, follow Option 2 (Ubuntu 24.04)**

## Build

Once ROS 2 is installed and sourced:

```bash
# Navigate to workspace
cd flowstate-ros-camera-drivers

# Build all packages
colcon build

# Or build specific packages
colcon build --packages-up-to flowstate_common
colcon build --packages-up-to flowstate_luxonis

# Build with verbose output (useful for debugging)
colcon build --event-handlers console_direct+

# Source the workspace after building
source install/setup.bash
```

## Quick Verification

```bash
# Verify the build succeeded
ls -la install/flowstate_common/lib/libflowstate_common.so

# Verify packages are discoverable
ros2 pkg list | grep flowstate

# Run the quick test
./quick_test.sh
```

## Camera-Specific Dependencies

### Luxonis OAK Camera

```bash
# Install depthai-ros
sudo apt install ros-jazzy-depthai-ros

# Or build from source:
# https://github.com/luxonis/depthai-ros
```

### Orbbec Camera

```bash
# Install OrbbecSDK and orbbec_camera package
# https://github.com/orbbec/OrbbecSDK
# https://github.com/orbbec/ros2_camera

# Typically:
sudo apt install ros-jazzy-orbbec-camera
```

## Troubleshooting

### "colcon: command not found"

**Solution**: Install colcon and source ROS 2:
```bash
sudo apt install python3-colcon-common-extensions
source /opt/ros/jazzy/setup.bash
```

### CMake version too old

**Solution**: Update CMake:
```bash
sudo apt install cmake
cmake --version  # Should be 3.22+
```

### "abseil_cpp_vendor" not found

**Solution**: Install abseil dependency:
```bash
sudo apt install ros-jazzy-abseil-cpp-vendor
```

### Missing OpenCV

**Solution**: Install OpenCV:
```bash
sudo apt install ros-jazzy-cv-bridge libopencv-dev
```

### Build fails with C++ errors

**Solution**: Ensure C++20 support:
```bash
gcc --version   # Should be 11+
g++ --version   # Should be 11+
```

If needed:
```bash
sudo apt install gcc-11 g++-11
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 100
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 100
```

## What Gets Built

After a successful build, you'll have:

### Shared Library
- `install/flowstate_common/lib/libflowstate_common.so` - Base classes and utilities

### Camera-Specific Libraries
- `install/flowstate_luxonis/lib/libflowstate_luxonis.so` - Luxonis adapter
- `install/flowstate_orbbec/lib/libflowstate_orbbec.so` - Orbbec adapter

### Headers
```
install/flowstate_common/include/flowstate_common/
  ├── camera_adapter_node.h      # Base adapter class
  ├── camera_spawner_node.h      # Base spawner class
  └── image_utils.h              # Image utilities
```

### Executables
- `install/flowstate_luxonis/lib/flowstate_luxonis/spawner_node` - Luxonis spawner
- `install/flowstate_luxonis/lib/flowstate_luxonis/adapter_node` - Luxonis adapter
- `install/flowstate_orbbec/lib/flowstate_orbbec/spawner_node` - Orbbec spawner
- `install/flowstate_orbbec/lib/flowstate_orbbec/adapter_node` - Orbbec adapter

## Next Steps

1. **Source the workspace**: `source install/setup.bash`
2. **Run tests**: See `TESTING.md`
3. **Add a new camera**: See `ADDING_NEW_CAMERA.md`
4. **Connect camera hardware** and run:
   ```bash
   ros2 run flowstate_luxonis spawner_node
   # or
   ros2 run flowstate_orbbec spawner_node
   ```

## Architecture Overview

See `README.md` for:
- Base class architecture
- How camera drivers are structured
- Design patterns used

## Additional Resources

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [ROS 2 Development Guide](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-Subscriber.html)
- `ADDING_NEW_CAMERA.md` - Guide for implementing new camera drivers
- `TESTING.md` - Comprehensive testing guide
