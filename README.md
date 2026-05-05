# flowstate-ros-camera-drivers

Welcome.

This repo is intended to contain Flowstate-compatible ROS adapter nodes for existing ROS camera drivers.

# Building

This repo is a collection of ROS packages, intended to build and run on ROS Jazzy.
As such, it needs to be in a [ROS workspace](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) in order to use `colcon` to build it.

At time of writing, it is necessary to use `rmw_zenoh` version 0.2.3 in order to interoperate with the Zenoh version and metadata formats used by the Intrinsic Platform, so it is necessary to check out and build `rmw_zenoh` in the workspace.

Here is an example command sequence to create such a workspace:

```
mkdir -p ros_cameras_ws/src
cd ros_cameras_ws/src
git clone ssh://git@github.com/intrinsic-dev/flowstate-ros-camera-drivers
git clone ssh://git@github.com/intrinsic-ai/sdk-ros
git clone https://github.com/codebot/rmw_zenoh -b morganquigley/jazzy_with_old_attachment_metadata_format && cd rmw_zenoh && git checkout 05cdda05a7c5e7c2871e6d85bfc9411546a529c4 && cd ..
```

Note: You will also need to clone the dependencies for your specific camera driver. Please check the specific driver READMEs for their clone commands, but first finish this guide here.

The resulting directory structure should look like this:
```
ros_cameras_ws/
└── src
    ├── flowstate-ros-camera-drivers
    ├── rmw_zenoh
    └── sdk-ros
```

ROS Jazzy expects to run on Ubuntu 24.04 LTS.
This can run conveniently on `gLinux` using `distrobox`.
```
# distrobox setup
sudo apt install distrobox
distrobox create -i ubuntu:24.04 -n ubuntu-24-04
distrobox enter ubuntu-24-04
```
For cameras which require OpenCL and GPU-support, follow the instructions below to generate a `distrobox` container with OpenCL and GPU-support:
```
sudo apt install distrobox
sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml
distrobox create --additional-flags "--gpus all" -i docker.io/nvidia/cuda:12.5.1-base-ubuntu24.04 -n ubuntu-24-04-nvidia

distrobox enter ubuntu-24-04-nvidia

sudo apt install ocl-icd-libopencl1
sudo mkdir -p /etc/OpenCL/vendors
echo "libnvidia-opencl.so.1" | sudo tee /etc/OpenCL/vendors/nvidia.icd
```

Assuming the distrobox is named `ubuntu-24-04` or `ubuntu-24-04-nvidia` and that the typical [desktop ROS Jazzy instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) have been followed, here are some packages to add:
```
# additional packages
sudo apt update

sudo apt install g++ libbz2-dev libzmq3-dev libczmq-dev nlohmann-json3-dev libprotobuf-dev protobuf-compiler
sudo apt install ros-jazzy-camera-info-manager ros-jazzy-image-publisher ros-jazzy-ament-cmake-vendor-package python3-colcon-common-extensions

curl https://sh.rustup.rs -sSf | sh
. "$HOME/.cargo/env"
```

Finally, let's build it!
```
cd ~/ros_cameras_ws
source /opt/ros/jazzy/setup.bash
colcon build

# if you want to skip building with other unused ros camera driver packages, you can use the following command:
colcon build --packages-above-and-dependencies <camera driver package name>
```

# Supported Drivers

* [Orbbec](flowstate_orbbec/README.md)
* [Luxonis](flowstate_luxonis/README.md)
* [Zivid](flowstate_zivid/README.md)

# Debugging builds

Sometimes there is just too much going on in parallel, and it's hard to sift through the console traffic. This invocation builds things one-at-a-time:
```
colcon build --event-handlers console_direct+ --executor sequential
```


# Disclaimer

This is not an officially supported Google product. This project is not
eligible for the [Google Open Source Software Vulnerability Rewards
Program](https://bughunters.google.com/open-source-security).

These drivers are designed to be compatible with the Intrinsic Platform. Use of the Intrinsic Platform is subject to the Intrinsic Terms of Service. Please review them here: [Intrinsic Platform Terms of Service URL](https://www.intrinsic.ai/legal/platform-terms)

# Ensenso details

**This camera is still experimental!**

To build and run the Ensenso driver locally follow these steps inside of the distrobox container:

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
   sudo pip3 install transforms3d --break-system-packages
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

4. **Build the Packages**:
   Navigate to the workspace root and build `flowstate_ensenso`.
   ```bash
   cd ~/ros_cameras_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --packages-above-and-dependencies flowstate_ensenso
   ```

5. **Run the Nodes and Scripts**:
   After the build, you can run the Ensenso camera nodes and Python scripts.
   ```bash
   source ~/ros_cameras_ws/install/setup.bash

   # To see available scripts and nodes
   ros2 run ensenso_camera <tab><tab>

   # To see available launch files
   ros2 launch ensenso_camera <tab><tab>
   ```
   To test the Flowstate specific node:
   ```bash
   ros2 run flowstate_ensenso ensenso_spawner_node
   ```
