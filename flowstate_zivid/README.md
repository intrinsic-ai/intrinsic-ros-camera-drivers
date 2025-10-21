# flowstate-ros-camera-drivers for zivid cameras

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
git clone https://github.com/codebot/OrbbecSDK_ROS2 --branch use_sdk_from_colcon
```

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
Assuming the distrobox is named `ubuntu-24-04` and that the typical [desktop ROS Jazzy instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) have been followed, here are some packages to add:

```
distrobox enter ubuntu-24-04
sudo apt install ros-jazzy-camera-info-manager ros-jazzy-image-publisher
```

# Zivid details

Follow this [Guide](https://support.zivid.com/en/latest/getting-started/software-installation.html) to install Zivid Core on your computer. Zivid SDK requires an OpenCL 1.2 compatible GPU with driver. Follow this [guide](https://support.zivid.com/en/latest/getting-started/software-installation/gpu/install-opencl-drivers-ubuntu.html) to install OpenCL drivers for your system.

Finally, let's build it!
```
cd ~/ros_cameras_ws
source /opt/ros/jazzy/setup.bash
colcon build
```

To test the camera functions locally, use following command to trigger capuring.

```
source install/setup.bash
ros2 run zivid_driver zivid_driver_main_test
```

If everything is set, we can start to build the service container for the zivid camera:

```
./build_service_bundle.sh
```

Sideload and install the service container in flowstate:
```
export SERVICE_BUNDLE=~/flowstate_ws/images/zivid_driver.bundle.tar 

export INTRINSIC_ORGANIZATION=<org name>

inctl cluster list --org $INTRINSIC_ORGANIZATION

export INTRINSIC_CONTEXT=<cluster id>

inctl service install --org $INTRINSIC_ORGANIZATION --cluster $INTRINSIC_CONTEXT $SERVICE_BUNDLE
```


