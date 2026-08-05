# Ensenso details

**This camera is still experimental!**

## Dependencies

To build the Ensenso driver, first ensure you have completed the core setup in the [main README](../README.md#building). Then, make sure you have the following Ensenso-specific repositories cloned in your workspace `src` directory:

1. **Clone the Ensenso ROS 2 Driver**:
   Clone the official Ensenso ROS driver repository into your workspace `src` directory.
   *Note: The `-c filter.lfs...` flags are used to disable Git LFS filters during the clone, ensuring that the checkout succeeds even if `git-lfs` is not installed.*
   ```bash
   cd ~/ros_cameras_ws/src
   git clone -c filter.lfs.smudge= -c filter.lfs.clean= -c filter.lfs.process= -c filter.lfs.required=false https://github.com/ensenso/ros_driver
   ```

2. **Prepare the ROS 2 Build**:
   Navigate to the cloned directory and run the prepare script. This ignores ROS1 packages and prepares ROS2 files.
   ```bash
   cd ~/ros_cameras_ws/src/ros_driver
   export ROS_VERSION=2
   ./.github/scripts/prepare_ros2_build.sh
   ```

3. **Install Ensenso SDK and Dependencies**:
   You can install the dependencies either by running the provided script or manually.

   **Option A (Recommended): Using the script**
   ```bash
   export ENSENSO_INSTALL=/opt/ensenso
   export ENSENSO_SDK_VERSION=4.3.905 # Version used in Dockerfile
   export ROS_VERSION=2
   export ROS_DISTRO=jazzy
   ./.github/scripts/install_external_dependencies.sh
   ```

   **Option B: Manual installation**
   If you already have the SDK installed (e.g., via `wget` and `dpkg`), you can install the remaining dependencies:
   ```bash
   sudo apt update
   sudo apt install -y libopencv-dev python3-opencv
   sudo apt install -y ros-jazzy-tf-transformations
   sudo apt install -y python3-transforms3d python3-retrying
   ```

   For xacro-based launch files, install these additional dependencies:
   ```bash
   sudo apt install -y ros-jazzy-joint-state-publisher-gui
   sudo apt install -y ros-jazzy-xacro
   ```

   **Step 3.C: Install remaining ROS dependencies**
   After installing the SDK, navigate back to the workspace root and run `rosdep` to install all missing ROS packages automatically. 
   *Note: Make sure you ran Step 2 first, so `rosdep` knows to ignore the source folders.*
   ```bash
   cd ~/ros_cameras_ws
   rosdep update
   rosdep install --from-paths src --ignore-src -y --skip-keys "OrbbecSDK"
   ```

## Testing locally

To test the camera functions locally, use the following command to start the camera driver (see the [main README](../README.md#building) for `distrobox` setup instructions):

```bash
distrobox enter ubuntu-24-04
cd ~/ros_cameras_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to flowstate_ensenso

source install/setup.bash

# To see available scripts and nodes
ros2 run ensenso_camera <tab><tab>

# To see available launch files
ros2 launch ensenso_camera <tab><tab>
```

To test the Flowstate specific node:
```bash
ros2 run flowstate_ensenso ensenso_spawner_node
```

## Building the Service Bundle

If everything is working locally, you can build the service container for the Ensenso camera **outside the distrobox container**:

```bash
cd ~/ros_cameras_ws
./src/flowstate-ros-camera-drivers/flowstate_ensenso/flowstate/build_service_bundle.sh
```

## Sideloading into Flowstate

To sideload and install the service container in Flowstate:
```bash
export SERVICE_BUNDLE=~/ros_cameras_ws/images/ensenso_driver.bundle.tar 

export INTRINSIC_ORGANIZATION=<org name>

inctl cluster list --org $INTRINSIC_ORGANIZATION

export INTRINSIC_CONTEXT=<cluster id>

inctl asset install --org $INTRINSIC_ORGANIZATION --cluster $INTRINSIC_CONTEXT $SERVICE_BUNDLE
```
