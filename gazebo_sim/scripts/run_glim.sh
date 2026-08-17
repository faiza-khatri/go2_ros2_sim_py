#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="$(realpath "$SCRIPT_DIR/../glim_config")"

echo "[glim] Using config from: $CONFIG_DIR"

  docker run \
  -it --rm \
  --net=host \
  --ipc=host \
  --pid=host \
  --gpus all \
  --privileged \
  -e ROS_DOMAIN_ID=10 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$CONFIG_DIR":/glim/config \
  -v /tmp/glim_dump:/tmp/dump \
  koide3/glim_ros2:humble_cuda12.2 \
  ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/glim/config
