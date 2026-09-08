#!/bin/bash
IMAGES_DIR=images
BUILDER_NAME=container-builder
mkdir -p $IMAGES_DIR
docker buildx inspect --builder $BUILDER_NAME || \
  docker buildx create --name="$BUILDER_NAME" --driver="docker-container"

echo "images
build
log" > ./.dockerignore
