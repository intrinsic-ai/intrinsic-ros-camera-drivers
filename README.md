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
git clone https://github.com/ros2/rmw_zenoh --branch 0.2.3
git clone https://github.com/codebot/OrbbecSDK_v2 OrbbecSDK --branch building_without_usb_on_linux
git clone https://github.com/codebot/OrbbecSDK_ROS2 --branch use_sdk_from_colcon_build
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
    ├── zivid-ros
    └── sdk-ros
```

ROS Jazzy expects to run on Ubuntu 24.04 LTS.
This can run conveniently on `gLinux` using `distrobox`.
```
# distrobox setup
sudo apt install distrobox
sudo usermod --add-subuids 200000-265535 --add-subgids 200000-265535 $USER && podman system migrate
distrobox create -i ubuntu:24.04 -n ubuntu-24-04
distrobox enter ubuntu-24-04

# additional packages
sudo apt update
sudo apt install ros-jazzy-camera-info-manager ros-jazzy-image-publisher ros-jazzy-ament-cmake-vendor-package
sudo apt install libzmq3-dev libczmq-dev nlohmann-json3-dev
sudo apt-get install libprotobuf-dev protobuf-compiler
```
For Zivid-related packages, please follow the [Zivid details](#zivid-details) section below.


Finally, let's build it!
```
cd ~/ros_cameras_ws
source /opt/ros/jazzy/setup.bash
colcon build

# if you want to skip building with unused ros camera driver packages, you can use the following command:
colcon build --packages-skip <camera driver package name>
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

```

`Zivid SDK` requires an OpenCL 1.2 compatible GPU with driver. Follow this [Guide](https://support.zivid.com/en/latest/getting-started/software-installation/gpu/install-opencl-drivers-ubuntu.html) to install OpenCL drivers inside the distrobox matching your system nvidia driver version. For example:
```
sudo apt install nvidia-driver-550
```

Finally, let's build it!
```
cd ~/ros_cameras_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-skip flowstate_orbbec
```

To test the camera functions locally, use the following command to trigger capturing.

```
source install/setup.bash
ros2 run flowstate_zivid zivid_driver_main
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
