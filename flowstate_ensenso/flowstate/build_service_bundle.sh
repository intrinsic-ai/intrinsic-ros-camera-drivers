#!/bin/bash
if [ ! -d "src/sdk-ros" ] || [ ! -d "src/flowstate-ros-camera-drivers" ]; then
  echo "This script must be run at the top of a Colcon workspace. See README for details"
  exit
fi

set -o errexit
set -o verbose

src/sdk-ros/scripts/setup_docker.sh
src/sdk-ros/scripts/build_container.sh --service_name ensenso_driver --service_package flowstate_ensenso --dockerfile src/flowstate-ros-camera-drivers/flowstate_ensenso/flowstate/Dockerfile.service
src/sdk-ros/scripts/build_bundle.sh --service_name ensenso_driver --service_package flowstate_ensenso
