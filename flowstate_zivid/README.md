# Zivid details

## Dependencies

To build the Zivid driver, first ensure you have completed the core setup in the [main README](../README.md). Then, make sure you have the following Zivid-specific repository cloned in your workspace `ros_cameras_ws/src` directory:

```bash
git clone https://github.com/zivid/zivid-ros.git
```

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
colcon build --packages-up-to flowstate_zivid
```

## Testing locally

To test the camera functions locally, use the following command to start camera driver:

```
source install/setup.bash
ros2 run flowstate_zivid zivid_driver_main
```
Open a second terminal to call ros2 services for testing the camera driver locally (see the [main README](../README.md#building) for `distrobox` setup instructions):
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
## Building the Service Bundle

If everything is set, we can start to build the service container for the zivid camera **outside the distrobox container**:

```
cd ~/ros_cameras_ws
./src/flowstate-ros-camera-drivers/flowstate_zivid/flowstate/build_service_bundle.sh
```

## Sideloading into Flowstate

To sideload and install the service container in flowstate:
```
export SERVICE_BUNDLE=~/ros_cameras_ws/images/zivid_driver.bundle.tar 

export INTRINSIC_ORGANIZATION=<org name>

inctl cluster list --org $INTRINSIC_ORGANIZATION

export INTRINSIC_CONTEXT=<cluster id>

inctl asset install --org $INTRINSIC_ORGANIZATION --cluster $INTRINSIC_CONTEXT $SERVICE_BUNDLE
```
