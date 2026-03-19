#!/bin/bash
set -o errexit
set -o verbose

SERVICE_NAME=$1
SERVICE_PACKAGE=$2
DOCKERFILE="src/flowstate-ros-camera-drivers/${SERVICE_PACKAGE}/flowstate/Dockerfile.flowstate_service"

src/sdk-ros/scripts/setup_docker.sh

echo "Testing build for ${SERVICE_NAME}..."

docker buildx build -t ${SERVICE_PACKAGE}:${SERVICE_NAME} \
  --file ${DOCKERFILE} \
  --build-arg="SERVICE_PACKAGE=${SERVICE_PACKAGE}" \
  --build-arg="SERVICE_NAME=${SERVICE_NAME}" \
  --build-arg="SERVICE_EXECUTABLE_NAME=${SERVICE_NAME}_main" \
  .
