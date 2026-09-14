#!/usr/bin/env bash
# Existing read-only Pi bridge owns the FC serial port; run this on the ground PC.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
assets="${repo}/data/processed/2026-09-05_gpu_inference_v01"
export ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-${repo}/code/deployment/raspberry_pi_readonly_stack/fastdds_pi_peer.xml}"
exec bash "${repo}/code/tools/run_with_camera_models.sh" \
  ros2 launch nexus_bringup live_approximate_localization.launch.py \
  detector_manifest:="${assets}/detector/detector_manifest.json" \
  superpoint_manifest:="${assets}/superpoint/superpoint_manifest.json" \
  rosbridge_port:=9091 "$@"
