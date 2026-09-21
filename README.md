# intrinsic-ros-camera-drivers

Welcome.

This repo contains Intrinsic-compatible ROS camera driver nodes and adapters.

# Building

This repo is a collection of ROS packages, intended to build and run on ROS Jazzy.
As such, it needs to be in a [ROS workspace](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) in order to use `colcon` to build it.

At time of writing, it is necessary to use `rmw_zenoh` version 0.2.3 in order to interoperate with the Zenoh version and metadata formats used by the Intrinsic Platform, so it is necessary to check out and build `rmw_zenoh` in the workspace.

Here is an example command sequence to create such a workspace:

```
mkdir -p ros_cameras_ws/src
cd ros_cameras_ws/src
git clone ssh://git@github.com/intrinsic-ai/intrinsic-ros-camera-drivers
git clone ssh://git@github.com/intrinsic-ai/sdk-ros
git clone https://github.com/codebot/rmw_zenoh -b morganquigley/jazzy_with_old_attachment_metadata_format && cd rmw_zenoh && git checkout 05cdda05a7c5e7c2871e6d85bfc9411546a529c4 && cd ..
```

Note: You will also need to clone the dependencies for your specific camera driver. Please check the specific driver READMEs for their clone commands, but first finish this guide here.

The resulting directory structure should look like this:
```
ros_cameras_ws/
└── src
    ├── intrinsic-ros-camera-drivers
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
colcon build --packages-up-to <camera driver package name>
```

# Supported Drivers

* [Orbbec](flowstate_orbbec/README.md)
* [Luxonis](flowstate_luxonis/README.md)
* [Zivid](flowstate_zivid/README.md)
* [Ensenso](flowstate_ensenso/README.md)

# Debugging builds

Sometimes there is just too much going on in parallel, and it's hard to sift through the console traffic. This invocation builds things one-at-a-time:
```
colcon build --event-handlers console_direct+ --executor sequential
```


---

## Documentation and related repositories

* [**Intrinsic Developer Community**](https://developer.intrinsic.ai): Complete guides, interactive tutorials, and API references.

---

## Contributing and community

Contributions are welcome! Please review:

* [CONTRIBUTING.md](CONTRIBUTING.md): Details on signing the Google Contributor License Agreement (CLA), community guidelines, C++20 coding standards, and pull request workflows.
* [SECURITY.md](SECURITY.md): Instructions for reporting security vulnerabilities.

---

## License

This project is licensed under the [Apache 2.0 License](LICENSE).

---

> **Disclaimer**: This is not an officially supported Google product. This project is not eligible for the [Google Open Source Software Vulnerability Rewards Program](https://bughunters.google.com/open-source-security).
>
> These drivers are designed to be compatible with the Intrinsic Platform. Use of the Intrinsic Platform is subject to the Intrinsic Terms of Service. Please review them here: [Intrinsic Platform Terms of Service](https://www.intrinsic.ai/legal/platform-terms).

---

### Trademark notice

"Intrinsic" and "Intrinsic Core" are trademarks of Intrinsic Innovation LLC. See [TRADEMARK.md](TRADEMARK.md) for usage guidelines.
