#!/usr/bin/env bash
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

IMAGES_DIR=./images
BUILDER_NAME=container-builder
ROS_DISTRO=jazzy
BUILD_CONTEXTS=()

while [[ $# -gt 0 ]]; do
  case $1 in
    --images_dir)
      IMAGES_DIR="$2"
      shift 2
      ;;
    --builder_name)
      BUILDER_NAME="$2"
      shift 2
      ;;
    --service_name)
      SERVICE_NAME="$2"
      shift 2
      ;;
    --service_package)
      SERVICE_PACKAGE="$2"
      shift 2
      ;;
    --skill_name)
      SKILL_NAME="$2"
      shift 2
      ;;
    --skill_package)
      SKILL_PACKAGE="$2"
      shift 2
      ;;
    --dockerfile)
      CUSTOM_DOCKERFILE="$2"
      shift 2
      ;;
    --dependencies)
      DEPENDENCIES="$2"
      shift 2
      ;;
    --ros_distro)
      ROS_DISTRO="$2"
      shift 2
      ;;
    --build-context)
      BUILD_CONTEXTS+=("$2")
      shift 2
      ;;
    -*|--*)
      echo "Unknown option $1"
      exit 1
      ;;
  esac
done

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

if [[ -n "$SERVICE_NAME" && -n "$SERVICE_PACKAGE" ]]; then
  mkdir -p "$IMAGES_DIR/$SERVICE_NAME"

  if [[ -n "$CUSTOM_DOCKERFILE" ]]; then
    DOCKERFILE="$CUSTOM_DOCKERFILE"
  else
    DOCKERFILE="$SCRIPT_DIR/../resources/Dockerfile.service"
  fi

  BUILD_ARGS=(
    -t "$SERVICE_PACKAGE:$SERVICE_NAME"
    --builder="$BUILDER_NAME"
    --output="type=docker,dest=$IMAGES_DIR/$SERVICE_NAME/$SERVICE_NAME.tar,compression=zstd,push=false,name=$SERVICE_PACKAGE:$SERVICE_NAME"
    --file "$DOCKERFILE"
    --build-arg="SERVICE_PACKAGE=$SERVICE_PACKAGE"
    --build-arg="SERVICE_NAME=$SERVICE_NAME"
    --build-arg="DEPENDENCIES=$DEPENDENCIES"
    --build-arg="SERVICE_EXECUTABLE_NAME=${SERVICE_NAME}_main"
    --build-arg="ROS_DISTRO=$ROS_DISTRO"
  )

  for context in "${BUILD_CONTEXTS[@]}"; do
    BUILD_ARGS+=(--build-context "$context")
  done

  docker buildx build "${BUILD_ARGS[@]}" .

elif [[ -n "$SKILL_NAME" && -n "$SKILL_PACKAGE" ]]; then
  mkdir -p "$IMAGES_DIR/$SKILL_NAME"

  if [[ -n "$CUSTOM_DOCKERFILE" ]]; then
    DOCKERFILE="$CUSTOM_DOCKERFILE"
  else
    DOCKERFILE="$SCRIPT_DIR/../resources/Dockerfile.skill"
  fi

  BUILD_ARGS=(
    -t "$SKILL_PACKAGE:$SKILL_NAME"
    --builder="$BUILDER_NAME"
    --output="type=docker,dest=$IMAGES_DIR/$SKILL_NAME/$SKILL_NAME.tar,compression=zstd,push=false,name=$SKILL_PACKAGE:$SKILL_NAME"
    --file "$DOCKERFILE"
    --build-arg="SKILL_PACKAGE=$SKILL_PACKAGE"
    --build-arg="SKILL_NAME=$SKILL_NAME"
    --build-arg="SKILL_EXECUTABLE_NAME=${SKILL_NAME}_main"
    --build-arg="ROS_DISTRO=$ROS_DISTRO"
    --build-arg="DEPENDENCIES=$DEPENDENCIES"
  )

  for context in "${BUILD_CONTEXTS[@]}"; do
    BUILD_ARGS+=(--build-context "$context")
  done

  docker buildx build "${BUILD_ARGS[@]}" .


fi
