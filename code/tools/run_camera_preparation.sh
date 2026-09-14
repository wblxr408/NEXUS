#!/usr/bin/env bash
# Run camera-only ROS ingress using the persistent project profile.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "${repo}/code/ros2_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID=20
export PYTHONPATH="${repo}/data/processed/2026-09-05_camera_models_v01/runtime:${PYTHONPATH:-}"
export OPENCV_FOR_THREADS_NUM=4
exec ros2 launch nexus_pi_readonly_ingress camera_preparation.launch.py "$@"
