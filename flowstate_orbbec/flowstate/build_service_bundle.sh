#!/bin/bash
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

if [ ! -d "src/sdk-ros" ] || [ ! -d "src/flowstate-ros-camera-drivers" ]; then
  echo "This script must be run at the top of a Colcon workspace. See README for details"
  exit
fi

set -o errexit

CAMERA_TYPE="orbbec"
UNDERLAY_DOCKERFILE="src/flowstate-ros-camera-drivers/ci_scripts/Dockerfile.${CAMERA_TYPE}_underlay"

LOCAL_TAG="intrinsic-dev-orbbec_gemini_driver-underlay:latest"

echo " Building local core underlay..."
docker build -t "intrinsic-dev-core-underlay:latest" -f "src/flowstate-ros-camera-drivers/ci_scripts/Dockerfile.core_underlay" src/flowstate-ros-camera-drivers/

echo " Building local underlay for $CAMERA_TYPE..."  
docker build -t "$LOCAL_TAG" -f "$UNDERLAY_DOCKERFILE" src/flowstate-ros-camera-drivers/
set -o verbose
src/sdk-ros/scripts/setup_docker.sh
src/sdk-ros/scripts/build_container.sh --service_name orbbec_gemini_driver --service_package flowstate_orbbec --dockerfile src/flowstate-ros-camera-drivers/flowstate_orbbec/flowstate/Dockerfile.flowstate_service
src/sdk-ros/scripts/build_bundle.sh --service_name orbbec_gemini_driver --service_package flowstate_orbbec
