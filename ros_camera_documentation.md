# Integration of third-party ROS-based cameras in Flowstate

This documentation provides a comprehensive guide for integrating third-party ROS-based cameras into Flowstate. To integrate a new ROS-based camera, you need to implement two key modules (highlighted in yellow in the workflow diagram) and build service containers around the existing components.

**Prerequisites**: We assume the camera has:
1. A vendor-provided camera SDK
2. A vendor-provided ROS2 Camera Driver that publishes information on ROS topics and responds to ROS service requests

![Workflow Overview](assets/workflow.png)

## Quick Start

**See [`example_camera/`](../example_camera/)** for a complete minimal template with working code you can copy and adapt for your camera.

## Architecture Overview

The integration consists of two main components:

1. **Camera Service Container**: Contains the ROS2 adapter driver that bridges the vendor's ROS driver with Flowstate's service interfaces
2. **Hardware Device Service Container**: Contains the camera CAD model and hardware device protos for the camera

**Network Configuration**: Camera network should be configured to use **link-local** addressing for automatic discovery and communication without manual IP configuration.

## Camera Service Container

### Required Flowstate ROS Services

All camera drivers must implement these standardized ROS services from [`snapshot_interfaces`](https://github.com/intrinsic-ai/sdk-ros/tree/80c542681486908e31d89a90e78395d26e5e48c9/snapshot_interfaces):

#### 1. `/cameras/discover` (SpawnerNode)
- **Type**: `snapshot_interfaces::srv::Discover`
- **Purpose**: Returns list of all physically connected cameras
- **Returns**: List of `DiscoveredCamera` messages (driver_type, camera_id)

#### 2. `~/<camera_id>/describe` (AdapterNode)
- **Type**: `snapshot_interfaces::srv::Describe`
- **Purpose**: Returns metadata about camera's available sensors
- **Returns**: List of `SensorInfo` messages (sensor_name, topic_name, sensor_type, transform to base link, cameraInfo)

#### 3. `~/<camera_id>/snapshot` (AdapterNode)
- **Type**: `snapshot_interfaces::srv::Snapshot`
- **Purpose**: Triggers capture and returns synchronized multi-modal sensor data
- **Returns**: `ImageSnapshot[]`, `PointCloud2Snapshot[]`, `ImuSnapshot[]`, ...

#### 4. Parameter Updates (AdapterNode)
- **Mechanism**: Standard ROS2 parameter callbacks via `add_on_set_parameters_callback()`
- **Purpose**: Runtime capture parameter configuration (exposure time, gain, etc.)
- **Pattern**: Translate Flowstate parameters to vendor-specific config

See [`snapshot_interfaces`](https://github.com/intrinsic-ai/sdk-ros/tree/80c542681486908e31d89a90e78395d26e5e48c9/snapshot_interfaces) for complete service and message definitions.

---

### Folder Structure

The camera service package follows this standardized structure:

```
flowstate_<vendor_name>/
├── CMakeLists.txt           # Build configuration
├── package.xml              # ROS package manifest
├── flowstate/
│   ├── Dockerfile.service                    # Multi-stage Docker build
│   ├── <vendor_name>_driver.manifest.textproto   # Service manifest
│   └── build_service_bundle.sh              # Build script
├── include/
│   └── flowstate_<vendor_name>/
│       ├── spawner_node.h   # Header for spawner node
│       └── adapter_node.h   # Header for adapter node
└── src/
    ├── spawner_node.cc      # Spawner node implementation
    ├── adapter_node.cc      # Adapter node implementation
    └── main.cc              # Main service entry point
```

**Examples**:
- **Minimal template**: [`example_camera/`](../example_camera/) - Start here for new integrations
- **Production examples**: `flowstate_zivid/`, `flowstate_orbbec/`, `flowstate_luxonis/`

---

### ROS2 Adapter Camera Driver for Flowstate

The adapter driver consists of three main components that work together to integrate the vendor's ROS camera driver with Flowstate.

### 1. Spawner Node

**Purpose**: Discovers physical cameras and manages adapter node lifecycle

**Key Responsibilities**:
- **Camera Discovery**: Automatically discovers all physically connected cameras on startup
- **Node Management**: Creates and manages one AdapterNode per discovered camera
- **Discovery Service**: Provides `/cameras/discover` service for Flowstate to query available cameras
- **Periodic Refresh**: Optionally refreshes camera list to detect newly connected/disconnected devices

**Implementation**: See [`example_camera/include/example_camera/spawner_node.h`](../example_camera/include/example_camera/spawner_node.h) and [`example_camera/src/spawner_node.cc`](../example_camera/src/spawner_node.cc) for a complete minimal implementation.

**Service Interface**:
- **Service Name**: `/cameras/discover`
- **Service Type**: `snapshot_interfaces::srv::Discover`
- **Response**: List of `DiscoveredCamera` messages containing:
  - `driver_type`: Camera driver identifier (e.g., "zivid", "orbbec")
  - `camera_id`: Unique identifier (typically serial number)

### 2. Adapter Node

**Purpose**: Acts as a bridge between the vendor's ROS camera driver and Flowstate's standardized interfaces

**Key Responsibilities**:
- **Vendor Driver Encapsulation**: Creates and manages the vendor's camera node instance
- **Configuration Updates**: Converts Flowstate parameter calls and pushes the changes directly to the vendor side.
- **Data Aggregation**: Subscribes to multiple ROS topics and aggregates synchronized data
- **Service Interface**: Implements Flowstate's `snapshot_interfaces` services:
  - `~/describe`: Returns camera sensor information
  - `~/snapshot`: Triggers capture and returns synchronized multi-modal data

**Implementation**: See [`example_camera/include/example_camera/adapter_node.h`](../example_camera/include/example_camera/adapter_node.h) and [`example_camera/src/adapter_node.cc`](../example_camera/src/adapter_node.cc) for a complete minimal implementation.

**Service Interfaces**:

1. **Describe Service** (`~/describe`)
   - Returns list of `SensorInfo` describing each sensor
   - Each `SensorInfo` includes:
     - `sensor_name`: e.g., "color", "depth", "normal"
     - `topic_name`: ROS topic where sensor data is published
     - `sensor_type`: IMAGE, DEPTH, NORMAL, POINT_CLOUD, IMU, etc.
     - `camera_t_sensor`: Transform from camera base to sensor frame
     - `info`: CameraInfo message (for image sensors)

2. **Snapshot Service** (`~/snapshot`)
   - Triggers camera capture
   - Waits for synchronized data from all subscribed topics
   - Returns aggregated multi-modal data:
     - `images[]`: Array of `ImageSnapshot` (image + camera_info pairs)
     - `point_clouds[]`: Array of `PointCloud2Snapshot`
     - `imus[]`: Array of `ImuSnapshot` (if applicable)

### 3. Main Service Entry Point

**File**: `main.cc`

**Purpose**: Application entry point that initializes and spins the SpawnerNode

**Implementation**: See [`example_camera/src/main.cc`](../example_camera/src/main.cc) for a simple main function that creates the SpawnerNode and spins it.

### Flowstate Service Protos

#### Service Manifest

**File**: `flowstate/<camera>_driver.manifest.textproto`

Defines the service metadata and deployment configuration. See [`example_camera/flowstate/example_driver.manifest.textproto`](../example_camera/flowstate/example_driver.manifest.textproto) for a template.

**Key Settings**:
- `host_network: true`: Essential for direct hardware communication
- `archive_filename`: Must match the Docker image export name

### `CMakeLists.txt` Configuration

See [`example_camera/CMakeLists.txt`](../example_camera/CMakeLists.txt) for a complete template showing:
- Required package dependencies
- Service manifest generation
- Library and executable build configuration
- Installation targets

### `package.xml` Configuration

See [`example_camera/package.xml`](../example_camera/package.xml) for a template showing all required dependencies.

### Dockerfile for Service Container

**File**: `flowstate/Dockerfile.service`

See [`example_camera/flowstate/Dockerfile.service`](../example_camera/flowstate/Dockerfile.service) for a complete multi-stage Docker build template.

**Key Dockerfile Considerations**:
1. **Multi-stage Build**: Minimizes final image size by separating build and runtime dependencies
2. **Hardware Access**: May need GPU support (NVIDIA CUDA base), USB access, or special permissions
3. **Vendor SDK**: Must be installed in both underlay (build) and result (runtime) stages
4. **RMW Zenoh**: Required for communication with Flowstate platform
5. **Vendored Libraries**: Copy critical libraries (protobuf, abseil) before cleaning build artifacts

### Build Script

See [`example_camera/flowstate/build_service_bundle.sh`](../example_camera/flowstate/build_service_bundle.sh) for a template build script.

**Output**: Produces `images/<camera>_driver.bundle.tar` ready for deployment

### Building and Testing Locally

1. **Build the ROS package**:
   ```bash
   cd ~/ros_cameras_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --packages-above-and-dependencies flowstate_<vendor>
   ```

2. **Test locally**:
   ```bash
   source install/setup.bash
   ros2 run flowstate_<vendor> <camera>_driver_main
   ```

3. **Test services** (in separate terminal):
   ```bash
   source install/setup.bash
   
   # Discover cameras
   ros2 service call /cameras/discover snapshot_interfaces/srv/Discover
   
   # Describe camera (replace <camera_id> with actual ID from discover)
   ros2 service call /<vendor>_<camera_id>/describe snapshot_interfaces/srv/Describe
   
   # Capture snapshot
   ros2 service call /<vendor>_<camera_id>/snapshot snapshot_interfaces/srv/Snapshot
   ```

### Building Service Container

**Run outside distrobox container**:

```bash
cd ~/ros_cameras_ws
./src/flowstate-ros-camera-drivers/flowstate_<vendor>/flowstate/build_service_bundle.sh
```

**Output**: `~/ros_cameras_ws/images/<camera>_driver.bundle.tar`

### Sideloading to Flowstate

Deploy the service container to your Flowstate cluster:

```bash
# Set environment variables
export SERVICE_BUNDLE=~/ros_cameras_ws/images/<camera>_driver.bundle.tar
export INTRINSIC_ORGANIZATION=<your_org_name>

# List available clusters
inctl cluster list --org $INTRINSIC_ORGANIZATION

# Set target cluster
export INTRINSIC_CONTEXT=<cluster_id>

# Install service
inctl service install \
  --org $INTRINSIC_ORGANIZATION \
  --cluster $INTRINSIC_CONTEXT \
  $SERVICE_BUNDLE
```

## Hardware Device Service Container

For simulation support, you need to create a hardware device service with the camera's 3D model.

### SDF File

**Purpose**: Simulation Definition Format file describing the camera's physical model and sensors

**Creating SDF from URDF**


### Hardware Device Protos

**Purpose**: Define the hardware device interface for Flowstate's resource management

## Code Examples

Reference implementations in this repository:

**Start Here**:
- **[`flowstate_common/`](./flowstate_common/)** - Base class to be implemented by new camera driver which takes care of defining snapshot_interfaces.

**Production Examples**:
- **`flowstate_zivid/`** - High-resolution 3D camera with GPU requirements and parameter handling
- **`flowstate_orbbec/`** - RGB-D camera with SDF simulation model
- **`flowstate_luxonis/`** - Wide-angle camera with custom undistortion

## Additional Resources

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Flowstate SDK Documentation](https://github.com/intrinsic-ai/sdk-ros)
- [snapshot_interfaces](https://github.com/intrinsic-ai/sdk-ros/tree/80c542681486908e31d89a90e78395d26e5e48c9/snapshot_interfaces) - Flowstate camera service interfaces