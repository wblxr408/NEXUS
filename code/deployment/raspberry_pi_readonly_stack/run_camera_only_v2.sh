#!/usr/bin/env bash
set -euo pipefail
stack_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
camera_dir="${NEXUS_CAMERA_DIR:-${HOME}/nexus_readonly_runs/live_camera_v2}"
camera_source_dir="${NEXUS_CAMERA_SOURCE_DIR:-${HOME}/NEXUS/code/deployment/raspberry_pi_e016_self_localization/app}"
export PYTHONPATH="${camera_source_dir}:${PYTHONPATH:-}"
exec python3 "${stack_dir}/capture_live_camera_v2.py" \
  --output-dir "${camera_dir}" --width 1640 --height 1232 --fps 30 --rotation 180 "$@"
