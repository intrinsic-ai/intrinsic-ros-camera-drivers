# RealSense details

The RealSense adapter composes the vendor's ROS 2 node, `realsense2_camera`
(apt `ros-jazzy-realsense2-camera`, [realsense-ros](https://github.com/realsenseai/realsense-ros)),
in-process as the rclcpp component it registers, so no vendor header is needed:
the `SpawnerNode` lists the cameras with librealsense and spawns one
`AdapterNode` per serial, named `realsense_<serial>`, which serves the Flowstate
`~/describe` and `~/snapshot` services from the driver's color, aligned-depth
and two infrared topics, and serves the driver's parameters as its own (see
[Settings](#settings)).

- driver node: `/realsense/camera_<serial>`, topics `color/image_raw`,
  `aligned_depth_to_color/image_raw`, `infra1/image_rect_raw`,
  `infra2/image_rect_raw`, each with its `camera_info` (and `tf_static`);
- Flowstate sensors, always described in this order (Flowstate's sensor ids):

  | id | sensor | stream | encoding |
  |---|---|---|---|
  | 1 | `rgb` (IMAGE) | color | rgb8 |
  | 2 | `depth` (DEPTH) | depth aligned to color | 32FC1 meters, from the driver's 16UC1 millimeters |
  | 3 | `ir_left` (IMAGE) | infra1, Y8 (rectified) | mono8 |
  | 4 | `ir_right` (IMAGE) | infra2, Y8 (rectified) | mono8 |

  The camera frame is the color optical frame: `rgb` and aligned `depth` report
  the identity `camera_t_sensor`, the infrared sensors the driver's static
  transforms;
- every stream starts on, as Flowstate describes the camera once when it opens
  it, before any setting; a snapshot holds the streams the settings leave on;
- the service config's `serials` lists the cameras to serve; empty (the
  default) serves every RealSense camera on the host:

  ```textproto
  [type.googleapis.com/realsense_driver.RealSenseDriverConfig] {
    serials: "123456789012"
  }
  ```

  The camera a Flowstate `CameraConfig` names is
  `identifier { ros { device_id: "<serial>" driver_type: "realsense" } }`.

Verified on an Intrinsic Core cell with a D405 (USB 3, color from the left
imager, firmware 5.12.14.100, `ir_right` 18.0 mm from the color frame along x)
and a D455 (separate RGB sensor at 1280x720, imagers 95 mm apart) running
together on one host with realsense-ros 4.58.4. Supports RealSense D400 models
with color, depth, and stereo IR (depth-only modules without color such as
D421/D430 are not supported).

## Settings

A Flowstate `CameraSetting` is a parameter call on `realsense_<serial>`, and
that node's parameter services are the driver's: a setting names a
realsense2_camera parameter verbatim, with the driver's type, the device's
ranges (the driver's parameter descriptors) and its live value.
`ros2 param list /realsense_<serial>` lists them all. Flowstate applies a
`CameraConfig`'s `camera_settings` on every capture, in order: it reads each
one and writes it when it differs.

```textproto
camera_settings { name: "enable_color" bool_value: false }
camera_settings { name: "enable_depth" bool_value: false }
camera_settings { name: "depth_module.infra_profile" string_value: "848x480x30" }
camera_settings { name: "depth_module.enable_auto_exposure" bool_value: false }
camera_settings { name: "depth_module.exposure" integer_value: 8500 }
```

Typical parameters (D405; check the names against `ros2 param list` for
another model):

| parameter | type | effect |
|---|---|---|
| `enable_color`, `enable_depth`, `enable_infra1`, `enable_infra2` | bool | the streams; a stream turned off is left out of the snapshot |
| `depth_module.depth_profile`, `depth_module.infra_profile` | string `<w>x<h>x<fps>` | resolution and rate; `ros2 param describe` lists the device's |
| `depth_module.color_profile` | string `<w>x<h>x<fps>` | the D405's color (it comes from the depth module; models with an RGB module use `rgb_camera.color_profile`) |
| `depth_module.enable_auto_white_balance`, `depth_module.white_balance` | bool, integer | the D405's color |
| `depth_module.enable_auto_exposure` | bool | |
| `depth_module.exposure` | integer, µs | with auto exposure off |
| `depth_module.gain` | integer | with auto exposure off |
| `decimation_filter.enable`, `spatial_filter.enable`, `temporal_filter.enable`, `hole_filling_filter.enable` | bool | depth post-processing |

- Use the oneof of the parameter's own type (`integer_value` for an integer
  parameter), and `string_value` for strings: Flowstate reads a parameter back
  as a string, so an `enumeration_value` never compares equal and is written
  on every capture.
- Depth is served aligned to color: it needs `enable_color` and
  `align_depth.enable` on as well as `enable_depth`.
- The D405's streams all come from its depth module and share one resolution
  and rate: change the profiles of the streams that are on together. Infrared
  at 1280x720 beside color and depth at 848x480 opens, but the device sends
  only corrupt infrared frames, so snapshots time out and the liveness check
  tears the node down after 30 s.
- A setting stays on the driver until something writes it again or the driver
  restarts (a new pod starts from the startup parameters): a config pins only
  what it names.
- realsense-ros applies a changed profile or format asynchronously when the
  stream restarts; the adapter triggers the restart at the next snapshot (after
  Flowstate finishes writing the batch of `camera_settings`) and waits for
  frames matching the active `*_profile` resolution and newer than the last
  settings change (FPS-only changes still have the upstream async restart
  window). Changing a stream's set or profile costs that capture a restart (the
  D405: 2.8 s against 0.9 s). The camera service's default capture timeout is
  500 ms: give a capture that turns a stream on or changes a profile a `timeout`
  of a few seconds (`CaptureRequest.timeout`), or it fails with DeadlineExceeded
  while the streams restart.
- Flowstate reads rgb8, mono8 and 32FC1: keep the infrared formats Y8 (the
  only format on `image_rect_raw`; unrectified Y16 would be mono16) and the
  color RGB8.
- Launch-time parameters (`serial_no`, `publish_tf`, `clip_distance`, ...) take
  no effect at run time.
- `capture_images`' `camera_settings_to_append` runs after the config's
  settings on every capture: a setting the config also names is written twice
  per capture.

## Dependencies

Beyond the [main README](../README.md#building): the RealSense SDK and ROS
driver from the ROS apt repository. No extra clone.

```bash
sudo apt install ros-jazzy-librealsense2 ros-jazzy-realsense2-camera
```

## Testing locally

```bash
cd ~/ros_cameras_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to flowstate_realsense
source install/setup.bash
ros2 run flowstate_realsense spawner_node
```

The spawner serves every RealSense camera on the host (the service config is
only read by the service's `flowstate_service`). In a second terminal:

```bash
ros2 service call /cameras/discover snapshot_interfaces/srv/Discover
ros2 service call /realsense_<serial>/describe snapshot_interfaces/srv/Describe
ros2 service call /realsense_<serial>/snapshot snapshot_interfaces/srv/Snapshot
```

## Building the Service Bundle

Outside the distrobox container, from the workspace root:

```bash
cd ~/ros_cameras_ws
./src/intrinsic-ros-camera-drivers/flowstate_realsense/flowstate/build_service_bundle.sh
```

The bundle is `images/realsense_driver/realsense_driver.bundle.tar`. On a real
cell the service runs privileged on the host's network: the apt librealsense
uses the V4L2 backend and opens the cameras through the host's `/dev/video*`
and `/dev/media*` nodes, numbered as they enumerate, which only a privileged
container sees.

## Sideloading into Flowstate

```bash
export SERVICE_BUNDLE=~/ros_cameras_ws/images/realsense_driver/realsense_driver.bundle.tar

export INTRINSIC_ORGANIZATION=<org name>

inctl cluster list --org $INTRINSIC_ORGANIZATION

export INTRINSIC_CONTEXT=<cluster id>

inctl asset install --org $INTRINSIC_ORGANIZATION --cluster $INTRINSIC_CONTEXT $SERVICE_BUNDLE
```
