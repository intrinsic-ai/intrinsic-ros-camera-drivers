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

src/sdk-ros/scripts/setup_docker.sh

CAMERA_TYPE="orbbec"
SERVICE_NAME="orbbec_gemini_driver"
SERVICE_PACKAGE="flowstate_orbbec"

# Detect if running in CI/presubmit
CONTEXT_ARGS=()
if [[ -n "$CI" || "$PRESUBMIT" == "true" ]]; then
  echo "Presubmit detected. Skipping slow underlay build stages, using ghcr.io images..."
  CONTEXT_ARGS+=(
    --build-context "core_underlay=docker-image://ghcr.io/${GITHUB_OWNER}/core-underlay:latest"
    --build-context "orbbec_underlay=docker-image://ghcr.io/${GITHUB_OWNER}/${CAMERA_TYPE}-underlay:latest"
  )
else
  echo "Running locally. Resolving and building all stages locally..."
fi

set -o verbose
src/flowstate-ros-camera-drivers/ci_scripts/build_container.sh \
  --builder_name container-builder \
  --service_name "$SERVICE_NAME" \
  --service_package "$SERVICE_PACKAGE" \
  --dockerfile "src/flowstate-ros-camera-drivers/${SERVICE_PACKAGE}/flowstate/Dockerfile.flowstate_service" \
  "${CONTEXT_ARGS[@]}"

if [[ -z "$CI" && "$PRESUBMIT" != "true" ]]; then
  src/sdk-ros/scripts/build_bundle.sh \
    --service_name "$SERVICE_NAME" \
    --service_package "$SERVICE_PACKAGE" \
    --manifest_path "src/flowstate-ros-camera-drivers/${SERVICE_PACKAGE}/flowstate/${SERVICE_NAME}.manifest.textproto"
fi
