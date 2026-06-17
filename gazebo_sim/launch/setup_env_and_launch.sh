#!/bin/bash

# Source the install setup
source "$(dirname "$0")/../../install/setup.bash"

# Get the gazebo_sim package path
pkg_path=$(ros2 pkg prefix gazebo_sim --share)

# Set GZ_MODEL_PATH
export GZ_MODEL_PATH="${pkg_path}/models:${GZ_MODEL_PATH}"

# Launch with the launch file
exec ros2 launch gazebo_sim "$@"
