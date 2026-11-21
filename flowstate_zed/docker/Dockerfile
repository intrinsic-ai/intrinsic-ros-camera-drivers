# Copyright 2026 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Build on the ZED Box, from the workspace root, with FLOWSTATE_IPC_ADDRESS set
# as compose.yaml describes:
#   docker compose -f src/intrinsic-ros-camera-drivers/flowstate_zed/docker/compose.yaml build

# The Stereolabs image for this L4T release and ZED SDK, pinned by digest.
ARG BASE_IMAGE=stereolabs/zed:5.5-devel-l4t-r39.2@sha256:05a8aa485a4661de8f674013378950af6d14cdb1ed4c625843955bf257a09a67
FROM ${BASE_IMAGE}

# The commits this driver was built and tested with.
ARG ZED_INTERFACES_COMMIT=8f86e35268881258cd3df98ab67afd2da5478e4d
ARG ZED_WRAPPER_COMMIT=f134486a814111b1c737a41211113dee41921776
ARG RMW_ZENOH_COMMIT=05cdda05a7c5e7c2871e6d85bfc9411546a529c4
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends curl ca-certificates \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo ${UBUNTU_CODENAME}) main" \
        > /etc/apt/sources.list.d/ros2.list \
    && rm -rf /var/lib/apt/lists/*
# As Stereolabs' build_wrapper.sh does: keep the image's own OpenCV.
RUN apt-mark hold libopencv-dev nvidia-opencv-dev nvidia-opencv \
    libopencv-python libopencv-samples opencv-licenses opencv-samples-data
# ros-jazzy-zstd-image-transport is here on purpose. The wrapper drops its raw
# image_transport publisher only if some other transport is installed; with
# none, it publishes every frame twice in-process. zstd is the one plugin that
# does not pull a second OpenCV.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake clang cargo git \
    ros-jazzy-ros-base ros-dev-tools ros-jazzy-ament-cmake-gtest \
    ros-jazzy-ament-cmake-vendor-package ros-jazzy-ament-cmake-auto \
    ros-jazzy-rclcpp-components ros-jazzy-image-transport \
    ros-jazzy-camera-info-manager ros-jazzy-diagnostic-updater \
    ros-jazzy-robot-localization ros-jazzy-nmea-msgs \
    ros-jazzy-geographic-msgs ros-jazzy-backward-ros ros-jazzy-stereo-msgs \
    ros-jazzy-visualization-msgs ros-jazzy-shape-msgs ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs ros-jazzy-zstd-image-transport \
    nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/ros/underlay
COPY src/sdk-ros src/sdk-ros
RUN git clone https://github.com/codebot/rmw_zenoh src/rmw_zenoh \
        -b morganquigley/jazzy_with_old_attachment_metadata_format \
    && git -C src/rmw_zenoh checkout ${RMW_ZENOH_COMMIT} \
    && git clone https://github.com/stereolabs/zed-ros2-interfaces src/zed-ros2-interfaces \
    && git -C src/zed-ros2-interfaces checkout ${ZED_INTERFACES_COMMIT} \
    && git clone https://github.com/stereolabs/zed-ros2-wrapper src/zed-ros2-wrapper \
    && git -C src/zed-ros2-wrapper checkout ${ZED_WRAPPER_COMMIT}
COPY src/intrinsic-ros-camera-drivers/flowstate_zed/patches/zed-ros2-wrapper.patch /tmp/
RUN git -C src/zed-ros2-wrapper apply /tmp/zed-ros2-wrapper.patch \
    && rm -f /tmp/zed-ros2-wrapper.patch
# RMW_IMPLEMENTATION must stay unset until rmw_zenoh_cpp is installed.
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --merge-install \
        --packages-up-to rmw_zenoh_cpp snapshot_interfaces abseil_cpp_vendor zed_components \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf build log src

WORKDIR /opt/ros/overlay
COPY src/intrinsic-ros-camera-drivers src/intrinsic-ros-camera-drivers
RUN . /opt/ros/jazzy/setup.sh \
    && . /opt/ros/underlay/install/setup.sh \
    && colcon build --merge-install --packages-up-to flowstate_zed \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && colcon test --merge-install --packages-select flowstate_zed \
    && colcon test-result --verbose \
    && rm -rf build log src

ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV RUST_LOG=info
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && source /opt/ros/overlay/install/setup.bash && exec /opt/ros/overlay/install/lib/flowstate_zed/spawner_node"]
