# Stereolabs ZED X (`flowstate_zed`)

## What it is

`flowstate_zed` is a Flowstate ROS 2 camera driver for Stereolabs GMSL2 cameras running on a Stereolabs ZED Box alongside a Flowstate IPC:

* Stereo models: `zed_x`, `zed_x_mini`, `zed_x_nano`
* Mono model: `zed_x_one_gs`

A single process (`spawner_node`) hosts one `SpawnerNode` and, for each discovered camera, one `AdapterNode` (`/<driver_type>_<serial>`, where `driver_type` is the model name `zed_x`, `zed_x_mini`, `zed_x_nano`, or `zed_x_one_gs`, and `camera_id` is the camera's decimal serial number) and one in-process Stereolabs wrapper component (`stereolabs::ZedCamera` for stereo, `stereolabs::ZedCameraOne` for mono) communicating over ROS 2 intra-process pub/sub.

### ROS distribution

ROS 2 Jazzy from Ubuntu 24.04 apt packages on `stereolabs/zed:5.5-devel-l4t-r39.2` is the only supported distribution.

## Requirements

* Stereolabs ZED Box with JetPack 7.2 / L4T R39.2 (`zed_x_daemon.service` and `nvargus-daemon.service` active on the host)
* ZED SDK 5.5 installed on the host (`/usr/local/zed/resources`, `/usr/local/zed/settings`)
* Docker with NVIDIA Container Toolkit (`nvidia` runtime registered) and the host user in the `docker` group

## Network

Jumbo frames (`MTU 9000`) on both ends of the 1 GbE link between the ZED Box (`enP8p1s0`) and the Flowstate IPC are required host configuration (not set by the driver) so that 13.8 MB stereo snapshot replies do not fragment excessively.

Set `MTU 9000` transiently on the ZED Box:

```bash
sudo ip link set dev <iface> mtu 9000
```

Make `MTU 9000` persistent across reboots with NetworkManager:

```bash
sudo nmcli connection modify <conn> 802-3-ethernet.mtu 9000
sudo nmcli connection up <conn>
```

On the ZED Box Mini test device, `<iface>` is `enP8p1s0`. Configure the matching 1 GbE interface on the Flowstate IPC to `MTU 9000` as well.

## Build and run

Create a workspace `ws` on the ZED Box containing `sdk-ros` and `intrinsic-ros-camera-drivers`:

```text
ws/
└── src/
    ├── intrinsic-ros-camera-drivers/
    └── sdk-ros/
```

(`rmw_zenoh`, `zed-ros2-interfaces`, and `zed-ros2-wrapper` are cloned at their pinned commits inside `Dockerfile` during the image build.)

From the workspace root (`ws/`), build the image:

```bash
FLOWSTATE_IPC_ADDRESS=<ipc-ip> docker compose -f src/intrinsic-ros-camera-drivers/flowstate_zed/docker/compose.yaml build
```

Start the driver container in the background:

```bash
FLOWSTATE_IPC_ADDRESS=<ipc-ip> docker compose -f src/intrinsic-ros-camera-drivers/flowstate_zed/docker/compose.yaml up -d
```

You can also store `FLOWSTATE_IPC_ADDRESS=<ipc-ip>` in `src/intrinsic-ros-camera-drivers/flowstate_zed/docker/.env` so `docker compose` commands do not need the variable on the command line.

### Stopping the driver and using Stereolabs tools

While `flowstate_zed` is running, `spawner_node` holds an exclusive Argus/GMSL lock on all connected ZED X cameras. Before launching Stereolabs host tools (`ZED_Explorer`, `ZED_Depth_Viewer`, `ZED_Diagnostic`) on the ZED Box, stop the container so the cameras are closed cleanly:

```bash
FLOWSTATE_IPC_ADDRESS=<ipc-ip> docker compose -f src/intrinsic-ros-camera-drivers/flowstate_zed/docker/compose.yaml stop
```

(Or `docker stop -t 30 flowstate_zed-flowstate_zed-1`; use `down` instead of `stop` to remove the stopped container.)

Because `compose.yaml` sets `restart: unless-stopped` and `stop_grace_period: 30s`, `stop` sends `SIGTERM` to `spawner_node`, waits for each camera to log `=== CAMERA CLOSED ===` (~3 s), and keeps the container stopped across reboots until started again:

```bash
FLOWSTATE_IPC_ADDRESS=<ipc-ip> docker compose -f src/intrinsic-ros-camera-drivers/flowstate_zed/docker/compose.yaml up -d
```

## Adding to Flowstate

Unlike USB or GigE camera drivers that run as service containers on the x86 Flowstate IPC, `flowstate_zed` runs on the external NVIDIA Jetson ZED Box and bridges to the Flowstate IPC's Zenoh router (`${FLOWSTATE_IPC_ADDRESS}:17447`).

In the Flowstate Asset Catalog, each camera model has a corresponding hardware device asset backed by `ai.intrinsic.ros_camera_service`:

| Model | Hardware Device Asset ID | `driver_type` |
|---|---|---|
| ZED X | `ai.intrinsic.zed_x` | `zed_x` |
| ZED X Mini | `ai.intrinsic.zed_x_mini` | `zed_x_mini` |
| ZED X Nano | `ai.intrinsic.zed_x_nano` | `zed_x_nano` |
| ZED X One GS | `ai.intrinsic.zed_x_one_gs` | `zed_x_one_gs` |

Once the `flowstate_zed` container is running on the ZED Box and the matching `ai.intrinsic.zed_x*` hardware device asset is installed on your Flowstate cluster, add the camera hardware device to your workcell and select the camera's discovered decimal serial number (`device_id`, e.g. `92232809`).

## Sensors and topics

* **Stereo (`zed_x`, `zed_x_mini`, `zed_x_nano`):** `Describe` returns two sensors in order:
  1. `left` (sensor id `1`, camera reference frame, identity `camera_t_sensor`) on `/zed/camera_<serial>/zed_node/left/color/raw/image`
  2. `right` (sensor id `2`, `camera_t_sensor` read once at adapter startup from `calibration_parameters_raw.stereo_transform` in `sl::COORDINATE_SYSTEM::IMAGE` and `sl::UNIT::METER` with self-calibration disabled) on `/zed/camera_<serial>/zed_node/right/color/raw/image`
* **Mono (`zed_x_one_gs`):** `Describe` returns one sensor:
  1. `rgb` (identity `camera_t_sensor`) on `/zed/camera_<serial>/zed_node/rgb/color/raw/image`

Matching raw `CameraInfo` messages are subscribed from `/zed/camera_<serial>/zed_node/{left,right,rgb}/color/raw/camera_info`.

Images returned by `Snapshot` are always uncompressed, unrectified `RGB8` at `1920x1200` (`step = 5760`, `6,912,000` bytes per image; ~13.8 MB per stereo snapshot reply, ~6.9 MB per mono snapshot reply). Raw frames stay intra-process via `SlImageAdapter` (`StampedSlMat`) with zero idle image copies; `BGR8` to `RGB8` conversion runs in a single pass only when a snapshot is requested. Zero image traffic crosses the 1 GbE link between snapshots.

## Parameters

Each `/<driver_type>_<serial>` adapter node (e.g. `/zed_x_nano_92232809`, `/zed_x_one_gs_301985430`) exposes twelve ROS 2 parameters:

| Parameter | Type | Unit | Range | Default | Description |
|---|---|---|---|---|---|
| `auto_exposure` | `bool` | — | `true`, `false` | `true` | Automatic exposure (`0.000028` to `0.030` s) and analog gain (`1.0` to `1.5` dB), with ISP digital gain pinned at `1`. |
| `exposure` | `double` | s | `0.000028` to `0.030` | `0.030` | Exposure time in seconds. Effective when `auto_exposure` is `false`. |
| `gain` | `double` | dB | `1.0` to `16.0` | `1.0` | Analog sensor gain in dB. Effective when `auto_exposure` is `false`. |
| `fps` | `double` | Hz | `15.0`, `30.0`, `60.0` | `30.0` | Grab and publish frame rate. Changing `fps` restarts the camera. |
| `exposure_compensation` | `int` | — | `0` to `100` | `50` | Exposure compensation (`50` is neutral). |
| `ae_antibanding` | `int` | — | `0` to `3` | `1` | Auto-exposure anti-banding: `0` (off), `1` (auto), `2` (50 Hz), `3` (60 Hz). Effective when `auto_exposure` is `true`. |
| `auto_white_balance` | `bool` | — | `true`, `false` | `true` | Automatic white balance. |
| `white_balance` | `int` | K | `2800` to `6500` (step `100`) | `4200` | White balance color temperature in K. Effective when `auto_white_balance` is `false`. |
| `gamma` | `int` | — | `1` to `9` | `8` | ISP gamma correction. |
| `saturation` | `int` | — | `0` to `8` | `4` | Color saturation. |
| `sharpness` | `int` | — | `0` to `8` | `4` | Image sharpness. |
| `denoising` | `int` | — | `0` to `100` | `50` | ISP denoising strength. |

Notes on parameter behavior:

* ISP digital gain is pinned at `1` (`video.auto_digital_gain: false` and `video.auto_digital_gain_range: [1, 1]`) in both automatic and manual modes (matching `common_mono.yaml` and `zedx_ar0234.isp`).
* In `auto_exposure: true` mode, automatic analog gain is capped at `1.5` dB (`1500` mdB) so the Argus AEC/AGC loop uses shutter time up to `30` ms before raising sensor gain. When `auto_exposure` is `false`, manual `exposure` and `gain` (`1.0` to `16.0` dB) are applied directly.
* All four `AR0234` models share the `0.000028` to `0.030` s exposure range and `1.0` dB minimum default analog gain (`1000` mdB).
* Changing `fps` destroys and recreates the in-process wrapper component while preserving all current video and ISP settings (~5.1 s to first snapshot on stereo, ~2.4 s on mono). `Describe` returns cached sensor calibration immediately and `Snapshot` waits up to `8` s for the first frame of the new wrapper. Setting `fps` to its current value is a no-op.
* In manual mode, the set-to-effect delay from a parameter set reply to a settled image level is ~0.45 s on mono (`zed_x_one_gs`) and ~0.5–1.0 s on stereo (`zed_x_nano`).
* `5` and `10` fps are not offered because the ZED SDK only supports `15`, `30`, and `60` fps at `HD1200` (`1920x1200`) and rounds other values to the nearest supported rate. Thinning frames in software at 15 fps would only increase snapshot wait latency without lengthening maximum exposure or reducing sensor load.

## Wrapper patch

The image builds `stereolabs/zed-ros2-wrapper` at commit `f134486a814111b1c737a41211113dee41921776` with `patches/zed-ros2-wrapper.patch` applied. The patch adds no class members, methods, or C++ exceptions and covers two areas across `zed_camera_component_main.cpp`, `zed_camera_one_component_main.cpp`, and `zed_camera_one_component_video.cpp`:

1. **Multi-camera process safety (8 edits across 6 diff hunks):**
   * **Clean stop on `exit(EXIT_FAILURE)` (4 edits):** Replaces the 4 reachable `exit(EXIT_FAILURE)` calls (`initNode()` camera open failure and grab-thread critical error in `ZedCamera` and `ZedCameraOne`) with clean component stops (`deInitNode()` on open failure; setting `mThreadStop` / `_threadStop` to leave the grab loop on critical grab error).
   * **Pre-shutdown callback lifetime guard (2 edits):** Captures `weak_from_this()` in the `add_pre_shutdown_callback` lambdas registered on the global ROS context so destroying a wrapper component on `fps` change or liveness recovery does not leave a dangling `this` pointer that segfaults during `rclcpp::shutdown()`.
   * **Defer `initServices()` until after `startCamera()` (2 edits):** Moves `initServices()` below the `startCamera()` check in both `initNode()` methods so a component that fails to open its camera does not advertise wrapper services with an uninitialized camera handle.
2. **`ZedCameraOne` dynamic video parameter fixes (4 diff hunks):**
   * **Map `video.denoising` in `handleExposure()`:** `ZedCameraOne` already registers and applies `video.denoising` in `applyExposureCompensationAndDenoising()`, so mapping it in dynamic dispatch avoids restarting the camera on denoising changes.
   * **Force manual scalar on `auto_* -> false`:** In `applyWhiteBalance()`, `applyExposure()`, and `applyAnalogGain()`, marks the paired manual scalar dirty when `auto_*` transitions to `false` so a manual value equal to the previously stored parameter is still written to the SDK.
   * **Preserve explicit auto ranges:** In `applyExposure()` and `applyAnalogGain()`, skips resetting the auto range to `[28, 30000]` / `[1000, 16000]` when explicit range bounds are already dirty in the same update (including startup overrides on frame 0).

### Rebasing the patch onto a newer wrapper commit

1. Clone `https://github.com/stereolabs/zed-ros2-wrapper`, check out the target commit, and compare `git grep -n "exit(EXIT_FAILURE)" -- zed_components` against the site classification table in the header of `patches/zed-ros2-wrapper.patch`.
2. Apply the three changes above to `zed_camera_component_main.cpp` and `zed_camera_one_component_main.cpp`. Confirm that `deInitNode()` remains idempotent when called in `initNode()` and again in the destructor, and that `git diff | grep -c "^+.*throw"` is `0`.
3. Export `git diff` to `flowstate_zed/patches/zed-ros2-wrapper.patch` (preserving the license and classification table header) and update `ZED_WRAPPER_COMMIT` (and `ZED_INTERFACES_COMMIT` if needed) in `flowstate_zed/docker/Dockerfile`.
4. Verify `git apply --check`, build `zed_components`, and verify on hardware that open failures do not terminate the process and container shutdown exits with code `0`.

## Limitations

* An unhandled crash (such as `SIGSEGV`) inside the ZED SDK or wrapper terminates `spawner_node` and restarts all cameras on the ZED Box via Docker's `unless-stopped` restart policy.
* Flowstate's ROS camera service stamps captured images with the IPC receive clock and ignores the ROS header stamp, so the capture timestamp seen in Flowstate is the IPC receive time.
* `capture_policy` and `timeout` in `Snapshot` requests are ignored; each call waits up to `5 s` (`8 s` during startup or a parameter-triggered restart) for a frame (or stamp-matched stereo pair) newer than the frame cached when the request arrived.
* When an `AdapterNode` exits after the 15 s liveness timeout and is recreated by `SpawnerNode`, its parameters return to defaults (`auto_exposure: true`, `fps: 30.0`).
* Camera discovery runs once at `SpawnerNode` startup because GMSL2 does not support hotplug and SDK enumeration can disturb active captures. A camera connected after startup requires a container restart; a camera disconnected while the driver is running remains in `/cameras/discover` and its adapter repeatedly respawns and fails until the camera is available again.
* Only one `spawner_node` may run per Flowstate IPC because `/cameras/discover` is a single global service name.
* **GMSL2 runtime cable unplug:** Unplugging a GMSL2 coax cable while streaming cuts Power-over-Coax (PoC) to that camera's `MAX9295` serializer and faults the shared `MAX9296` deserializer / Argus CSI channel on the ZED Box. While a wedged `nvargus-daemon` from a software crash is recovered automatically at container startup via `sl::Camera::reboot(sl::INPUT_TYPE::GMSL)`, a camera that lost PoC power requires `sudo systemctl restart zed_x_daemon.service` (or a host reboot) after replugging to reprogram its serializer over I2C.

## Tested hardware

* Tested on `zed_x_nano` (serial `92232809`) and `zed_x_one_gs` (serial `301985430`) on a Stereolabs ZED Box Mini (NVIDIA Jetson Orin NX 16 GB, JetPack 7.2.1 / L4T R39.2.1, ZED SDK 5.5.0).
* **`zed_x` and `zed_x_mini` were not tested on hardware.**
