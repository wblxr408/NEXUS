#!/usr/bin/env bash
# GPU model defaults for the existing read-only localization launch.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
assets="${repo}/data/processed/2026-09-05_gpu_inference_v01"
exec bash "${repo}/code/tools/run_with_camera_models.sh" \
  ros2 launch nexus_bringup dual_localization.launch.py \
  detector_manifest:="${assets}/detector/detector_manifest.json" \
  superpoint_manifest:="${assets}/superpoint/superpoint_manifest.json" "$@"
