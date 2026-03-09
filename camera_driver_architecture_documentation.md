# Integration of Third-Party ROS 2 Cameras in Flowstate

This repository provides a comprehensive guide and base architecture for integrating third-party ROS 2 cameras into Flowstate. 

To integrate a new ROS 2 camera, you need to implement two key modules and build service containers around the existing components.

**Prerequisites**: We assume the target camera has:
1. A vendor-provided C++ API/SDK.
2. A vendor-provided ROS 2 Camera Driver that publishes information on ROS topics and responds to ROS service requests.

-----

## Architecture Overview

The integration consists of two main components:

1. **Camera Service Container**: Contains the ROS 2 adapter driver that bridges the vendor's ROS driver with Flowstate's service interfaces.
2. **Hardware Device Service Container**: Contains the camera CAD model and hardware device protos for the camera.

> **Network Configuration**: Camera networks should be configured to use **link-local** addressing for automatic discovery and communication without manual IP configuration.

-----

## Camera Service Container

### The `flowstate_common` Base Classes
To ensure standardization and prevent code duplication, all camera integrations must utilize the base classes provided in the `flowstate_common` package. These base classes automatically handle ROS service hosting, thread safety, and lifecycle management.

### Required Flowstate ROS Services
Flowstate relies on standardized ROS services from [`snapshot_interfaces`](https://github.com/intrinsic-ai/sdk-ros/tree/80c542681486908e31d89a90e78395d26e5e48c9/snapshot_interfaces). These are largely managed for you by the base classes, but you must implement the hardware logic to populate their specific message types:

#### 1. `/cameras/discover` (Managed by `BaseSpawnerNode`)
*   **Type**: `snapshot_interfaces::srv::Discover`
*   **Purpose**: Returns a list of all physically connected cameras.
*   **Returns**: An array of `DiscoveredCamera` messages containing:
    *   `driver_type`: String identifier (e.g., "zivid", "luxonis")
    *   `camera_id`: Unique string identifier (typically the hardware serial number)

#### 2. `~/<camera_id>/describe` (Implemented via `BaseAdapterNode`)
*   **Type**: `snapshot_interfaces::srv::Describe`
*   **Purpose**: Returns metadata about a specific camera's available sensors.
*   **Returns**: An array of `SensorInfo` messages containing:
    *   `sensor_name`: Identifying name (e.g., "color", "depth", "normal")
    *   `topic_name`: The ROS topic where this sensor's data is actively published
    *   `sensor_type`: Enumeration (IMAGE, DEPTH, POINT_CLOUD, IMU, etc.)
    *   `camera_t_sensor`: The static transform from the sensor frame to the camera base frame
    *   `info`: Standard `sensor_msgs/CameraInfo`

#### 3. `~/<camera_id>/snapshot` (Implemented via `BaseAdapterNode`)
*   **Type**: `snapshot_interfaces::srv::Snapshot`
*   **Purpose**: Triggers a hardware capture and returns synchronized multi-modal sensor data.
*   **Returns**: Aggregated arrays of synchronized data structures:
    *   `images`: Array of `ImageSnapshot` messages (image + camera_info pairs)
    *   `point_clouds`: Array of `PointCloud2Snapshot` messages
    *   `imus`: Array of `ImuSnapshot` messages

### Folder Structure
A new camera service package should follow this standardized structure:

```text
flowstate_<vendor_name>/
├── CMakeLists.txt           # Build configuration
├── package.xml              # ROS package manifest
├── flowstate/
│   ├── Dockerfile.service                    # Multi-stage Docker build
│   ├── <vendor_name>_driver.manifest.textproto   # Service manifest
│   └── build_service_bundle.sh              # Build script
├── include/
│   └── flowstate_<vendor_name>/
│       ├── spawner_node.h   # Inherits from flowstate_common::BaseSpawnerNode
│       └── adapter_node.h   # Inherits from flowstate_common::BaseAdapterNode
└── src/
    ├── spawner_node.cc      # Hardware discovery logic
    ├── adapter_node.cc      # Vendor data bridging logic
    └── main.cc              # Main service entry point
```

-----

## ROS 2 Adapter Camera Driver Implementation

To integrate a new camera, you will implement three main components.

### 1\. Spawner Node

**Purpose**: Discovers physical hardware and instantiates Adapter Nodes.  
**Implementation**: Inherit from `flowstate_common::BaseSpawnerNode`. The base class completely owns the ROS discovery service and the background polling timer. You only need to provide the vendor-specific SDK logic by implementing two pure virtual methods:

*   `GetSerials()`: Queries the vendor SDK and returns a `std::vector<std::string>` of currently connected hardware serials.
*   `SpawnNodes(serials)`: Instantiates your specific `AdapterNode` for any newly discovered serial numbers.

### 2\. Adapter Node

**Purpose**: Acts as a bridge between the vendor's ROS camera driver and Flowstate's standardized interfaces for a single physical camera.  
**Implementation**: Inherit from `flowstate_common::BaseAdapterNode`.

*   **Vendor Driver Encapsulation**: Instantiates and manages the vendor's actual ROS node (or SDK runtime).
*   **Configuration Updates**: Listens to Flowstate parameter changes via `add_on_set_parameters_callback()` and pushes them to the vendor side (e.g., exposure time, gain).
*   **Service Fulfillment**: Fulfills `Describe` and `Snapshot` requests by subscribing to vendor topics, aggregating synchronized data, and returning it in Flowstate formats.

### 3\. Main Service Entry Point (`main.cc`)

**Purpose**: Application entry point that initializes ROS 2, instantiates your derived `SpawnerNode`, and spins it.

## Flowstate Service Protos & Docker

### Service Manifest (`flowstate/<vendor>_driver.manifest.textproto`)

Defines the service metadata and deployment configuration.

*   `host_network: true`: Essential for direct hardware communication and automatic discovery.
*   `archive_filename`: Must match the Docker image export name.

### Dockerfile (`flowstate/Dockerfile.service`)

*   **Multi-stage Build**: Minimizes final image size by separating build and runtime dependencies
*   **Hardware Access**: May need GPU support (NVIDIA CUDA base), USB access, or special permissions
*   **Vendor SDK**: Must be installed in both underlay (build) and result (runtime) stages
*   **RMW Zenoh**: Required for communication with Flowstate platform
*   **Vendored Libraries**: Copy critical libraries (protobuf, abseil) before cleaning build artifacts

-----

## Building and Testing Locally

**1. Build the ROS package:**

```bash
cd ~/ros_cameras_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to flowstate_<vendor>
```

**2. Test locally:**

```bash
source install/setup.bash
ros2 run flowstate_<vendor> <vendor>_driver_main
```

**3. Test services (in a separate terminal):**

```bash
source install/setup.bash

# Discover cameras
ros2 service call /cameras/discover snapshot_interfaces/srv/Discover

# Describe camera (replace <camera_id> with actual ID from discover)
ros2 service call /<vendor>_<camera_id>/describe snapshot_interfaces/srv/Describe

# Capture snapshot
ros2 service call /<vendor>_<camera_id>/snapshot snapshot_interfaces/srv/Snapshot
```

## Sideloading to Flowstate

Deploy the built service container to your Flowstate cluster:

```bash
# Set environment variables
export SERVICE_BUNDLE=~/ros_cameras_ws/images/<vendor>_driver.bundle.tar
export INTRINSIC_ORGANIZATION=<your_org_name>

# Set target cluster
export INTRINSIC_CONTEXT=<cluster_id>

# Install service
inctl asset install \
  --org $INTRINSIC_ORGANIZATION \
  --cluster $INTRINSIC_CONTEXT \
  $SERVICE_BUNDLE
```

-----

## Code Examples

When creating a new integration, rely on the base classes and reference the existing production drivers for guidance:

*   **`flowstate_common/`** - Contains `BaseSpawnerNode` and `BaseAdapterNode`. Start here to understand the required overrides.
*   **`flowstate_zivid/`** - Good reference for custom C++ SDK requirements and parameter handling.
*   **`flowstate_orbbec/`** - Good reference for standard RGB-D camera integration and SDF simulation models.
*   **`flowstate_luxonis/`** - Good reference for wide-angle cameras and IP/network device discovery.

## Additional Resources

*   [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
*   [Flowstate SDK Documentation](https://github.com/intrinsic-ai/sdk-ros)
*   [snapshot\_interfaces](https://github.com/intrinsic-ai/sdk-ros/tree/80c542681486908e31d89a90e78395d26e5e48c9/snapshot_interfaces) - Flowstate camera service interfaces
