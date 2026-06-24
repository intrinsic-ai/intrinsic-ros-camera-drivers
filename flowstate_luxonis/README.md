# Luxonis

## Dependencies

To build the Luxonis driver, first ensure you have completed the core setup in the [main README](../README.md). Then, make sure you have the following Luxonis-specific repositories cloned in your workspace `ros_cameras_ws/src` directory:

```bash
git clone https://github.com/codebot/depthai-core --branch kilted && cd depthai-core && git submodule update --init --recursive && cd ..
git clone https://github.com/codebot/depthai-ros --branch mq/adjust_export_dependencies
```

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

## Testing locally

To test the camera functions locally, use the following command to start the camera driver:

```bash
cd ~/ros_cameras_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-above-and-dependencies flowstate_luxonis

source install/setup.bash
ros2 run flowstate_luxonis spawner_node
```

Open a second terminal to call ros2 services for testing the camera driver locally (see the [main README](../README.md#building) for `distrobox` setup instructions):
```bash
distrobox enter ubuntu-24-04
cd ~/ros_cameras_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# discover
ros2 service call /cameras/discover snapshot_interfaces/srv/Discover

# snapshot
ros2 service call /luxonis_<serial_id>/snapshot snapshot_interfaces/srv/Snapshot

# describe
ros2 service call /luxonis_<serial_id>/describe snapshot_interfaces/srv/Describe

# see all available ros2 services
ros2 service list
```

## Building the Service Bundle

If everything is working locally, you can build the service container for the Luxonis camera **outside the distrobox container**:

```bash
cd ~/ros_cameras_ws
./src/flowstate-ros-camera-drivers/flowstate_luxonis/flowstate/build_service_bundle.sh
```

## Sideloading into Flowstate

To sideload and install the service container in Flowstate:
```bash
export SERVICE_BUNDLE=~/ros_cameras_ws/images/luxonis_driver.bundle.tar 

export INTRINSIC_ORGANIZATION=<org name>

inctl cluster list --org $INTRINSIC_ORGANIZATION

export INTRINSIC_CONTEXT=<cluster id>

inctl asset install --org $INTRINSIC_ORGANIZATION --cluster $INTRINSIC_CONTEXT $SERVICE_BUNDLE
```
