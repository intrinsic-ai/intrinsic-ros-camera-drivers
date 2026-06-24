# Orbbec details

## Dependencies

To build the Orbbec driver, first ensure you have completed the core setup in the [main README](../README.md). Then, make sure you have the following Orbbec-specific repositories cloned in your workspace `ros_cameras_ws/src` directory:

```bash
git clone https://github.com/codebot/OrbbecSDK_v2 OrbbecSDK --branch colcon_compatible_install
git clone https://github.com/codebot/OrbbecSDK_ROS2 --branch use_sdk_from_colcon_build
```

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

## Testing locally

To test the camera functions locally, use the following command to start the camera driver:

```bash
cd ~/ros_cameras_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-above-and-dependencies flowstate_orbbec

source install/setup.bash
ros2 run flowstate_orbbec spawner_node
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
ros2 service call /orbbec_<serial_id>/snapshot snapshot_interfaces/srv/Snapshot

# describe
ros2 service call /orbbec_<serial_id>/describe snapshot_interfaces/srv/Describe

# see all available ros2 services
ros2 service list
```

## Building the Service Bundle

If everything is working locally, you can build the service container for the Orbbec camera **outside the distrobox container**:

```bash
cd ~/ros_cameras_ws
./src/flowstate-ros-camera-drivers/flowstate_orbbec/flowstate/build_service_bundle.sh
```

## Sideloading into Flowstate

To sideload and install the service container in Flowstate:
```bash
export SERVICE_BUNDLE=~/ros_cameras_ws/images/orbbec_gemini_driver.bundle.tar 

export INTRINSIC_ORGANIZATION=<org name>

inctl cluster list --org $INTRINSIC_ORGANIZATION

export INTRINSIC_CONTEXT=<cluster id>

inctl asset install --org $INTRINSIC_ORGANIZATION --cluster $INTRINSIC_CONTEXT $SERVICE_BUNDLE
```
