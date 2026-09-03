#!/usr/bin/env bash
# Source this file from a shell before building, testing, or running NEXUS ROS 2.

if [ -z "${BASH_SOURCE[0]:-}" ] || [ "${BASH_SOURCE[0]}" = "$0" ]; then
  echo "Please source this file: source code/ros2_ws/scripts/activate_humble.sh" >&2
  exit 1
fi

# WSL may inherit TMP/TEMP values that point to a mounted Windows directory.
# pytest's capture setup can fail there during ament's CMake configure check.
export TMPDIR=/tmp

source /opt/ros/humble/setup.bash

