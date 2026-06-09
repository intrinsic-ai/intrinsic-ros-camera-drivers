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
git clone ssh://git@github.com/intrinsic-ai/sdk-ros && cd sdk-ros && git checkout 46960b1593447ddfa2e6d1037b37d1dd228f3014 && cd ..
git clone https://github.com/codebot/rmw_zenoh -b morganquigley/jazzy_with_old_attachment_metadata_format && cd rmw_zenoh && git checkout 05cdda05a7c5e7c2871e6d85bfc9411546a529c4 && cd ..
git clone https://github.com/codebot/OrbbecSDK_v2 OrbbecSDK --branch colcon_compatible_install
git clone https://github.com/codebot/OrbbecSDK_ROS2 --branch use_sdk_from_colcon_build
git clone https://github.com/codebot/depthai-core --branch kilted && cd depthai-core && git submodule update --init --recursive && cd ..
git clone https://github.com/codebot/depthai-ros --branch mq/adjust_export_dependencies
git clone https://github.com/zivid/zivid-ros.git
```

The resulting directory structure should look like this:
```
ros_cameras_ws/
└── src
    ├── depthai-core
    ├── depthai-ros
    ├── flowstate-ros-camera-drivers
    ├── OrbbecSDK
    ├── OrbbecSDK_ROS2
    ├── rmw_zenoh
    ├── zivid-ros
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
For Zivid-related packages, please follow the [Zivid details](#zivid-details) section below.


Finally, let's build it!
```
cd ~/ros_cameras_ws
source /opt/ros/jazzy/setup.bash
colcon build

# if you want to skip building with other unused ros camera driver packages, you can use the following command:
colcon build --packages-above-and-dependencies <camera driver package name>
```

# Orbbec details

Orbbec camera models are supplied by the vendor as Xacro URDF and several STL files.
To create a single SDF of an Orbbec camera, such as the Gemini 335 Le:
```
ros2 run xacro xacro ../src/OrbbecSDK_ROS2/orbbec_description/urdf/gemini335Le.urdf.xacro > orbbec_gemini_335le.urdf
gz sdf -p orbbec_gemini_335le.urdf > orbbec_gemini_335le.sdf
cp orbbec_gemini_335le.sdf ../install/orbbec_description/share/orbbec_description/
gz sim ../src/flowstate-ros-camera-drivers/flowstate_orbbec/model/minimal_world.sdf
```
That will produce `orbec_gemini_335le/meshes/orbbec_gemini_335le.dae` which is great, but has incredible detail, and is 44 MB.
To reduce this size drastically, load it into Blender 4.5, select "node 1", then Mesh..CleanUp..MergeByDistance, using something like 0.5mm, and export just "node 1" as a `.dae`.
The resulting size of this operation is around 1.9 MB.

# Luxonis

Although Luxonis releases a binary build of their software for ROS Jazzy, we
need a newer version (v3) to correctly handle undistortion of their wide-angle
cameras. This is released as binary for Kilted, but we will build it from
source on Jazzy.

```
colcon build --packages-up-to flowstate_luxonis --merge-install --cmake-args -DBUILD_SHARED_LIBS=ON
```

## Firmware updates

To update the firmware:
```
mkdir ~/luxonis
cd ~/luxonis
git clone https://github.com/luxonis/depthai-python.git
virtualenv venv
source venv/bin/activate
cd depthai-python/utilities
python3 install_requirements.py
python3 device_manager.py
```
Then click "Specify IP", and type an IP address on the local network, such as `192.168.1.203`

# Debugging builds

Sometimes there is just too much going on in parallel, and it's hard to sift through the console traffic. This invocation builds things one-at-a-time:
```
colcon build --event-handlers console_direct+ --executor sequential
```

# Zivid details

Follow this [Guide](https://support.zivid.com/en/latest/getting-started/software-installation.html) to install `Zivid Core 2.17.0` inside distrobox container.

```
wget \
https://downloads.zivid.com/sdk/releases/2.17.0+5fc9f05e-1/u24/amd64/zivid_2.17.0+5fc9f05e-1_amd64.deb \
https://downloads.zivid.com/sdk/releases/2.17.0+5fc9f05e-1/u24/amd64/zivid-studio_2.17.0+5fc9f05e-1_amd64.deb \
https://downloads.zivid.com/sdk/releases/2.17.0+5fc9f05e-1/u24/amd64/zivid-tools_2.17.0+5fc9f05e-1_amd64.deb \
https://downloads.zivid.com/sdk/releases/2.17.0+5fc9f05e-1/u24/amd64/zivid-genicam_2.17.0+5fc9f05e-1_amd64.deb

sudo apt update
sudo apt install ./*.deb

rm *.deb
```

Finally, let's build it!
```
cd ~/ros_cameras_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-above-and-dependencies flowstate_zivid
```

To test the camera functions locally, use the following command to start camera driver:

```
source install/setup.bash
ros2 run flowstate_zivid zivid_driver_main
```
Open a second terminal to call ros2 services for testing camera driver locally:
```
distrobox enter ubuntu-24-04-nvidia
cd ~/ros_cameras_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# discover
ros2 service call /cameras/discover snapshot_interfaces/srv/Discover

# snapshot
ros2 service call /zivid_<serial_id>/snapshot snapshot_interfaces/srv/Snapshot

# describe
ros2 service call /zivid_<serial_id>/describe snapshot_interfaces/srv/Describe

# see all available ros2 services
ros2 service list
```
If everything is set, we can start to build the service container for the zivid camera **outside the distrobox container**:

```
cd ~/ros_cameras_ws
./src/flowstate-ros-camera-drivers/flowstate_zivid/flowstate/build_service_bundle.sh
```

Sideload and install the service container in flowstate:
```
export SERVICE_BUNDLE=~/ros_cameras_ws/images/zivid_driver.bundle.tar

export INTRINSIC_ORGANIZATION=<org name>

inctl cluster list --org $INTRINSIC_ORGANIZATION

export INTRINSIC_CONTEXT=<cluster id>

inctl service install --org $INTRINSIC_ORGANIZATION --cluster $INTRINSIC_CONTEXT $SERVICE_BUNDLE
```

# Disclaimer

This is not an officially supported Google product. This project is not
eligible for the [Google Open Source Software Vulnerability Rewards
Program](https://bughunters.google.com/open-source-security).

These drivers are designed to be compatible with the Intrinsic Platform. Use of the Intrinsic Platform is subject to the Intrinsic Terms of Service. Please review them here: [Intrinsic Platform Terms of Service URL](https://www.intrinsic.ai/legal/platform-terms)
