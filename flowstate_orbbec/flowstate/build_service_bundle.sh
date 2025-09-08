#!/bin/bash
if [ ! -d "src/sdk-ros" ] || [ ! -d "src/flowstate-ros-camera-drivers" ]; then
  echo "This script must be run at the top of a Colcon workspace. See README for details"
  exit
fi

set -o errexit
set -o verbose
src/sdk-ros/scripts/build_container.sh --service_name orbbec_driver --service_package flowstate_orbbec --dockerfile src/flowstate-ros-camera-drivers/flowstate_orbbec/flowstate/Dockerfile.flowstate_service
src/sdk-ros/scripts/build_bundle.sh --service_name orbbec_driver --service_package flowstate_orbbec
