#!/usr/bin/env bash
# Source ROS and use the validated project-local model runtime for one command.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "${repo}/code/ros2_ws/install/setup.bash"
set -u
runtime="${repo}/data/processed/2026-09-05_camera_models_v01/runtime"
test -d "${runtime}/cv2" || { echo "Missing project camera-model runtime" >&2; exit 1; }
gpu_runtime="${repo}/data/processed/2026-09-05_gpu_inference_v01/runtime"
test -d "${gpu_runtime}/torch" || { echo "Missing project CUDA runtime" >&2; exit 1; }
export PYTHONPATH="${gpu_runtime}:${runtime}:${PYTHONPATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"
export OPENCV_FOR_THREADS_NUM=4
exec "$@"
