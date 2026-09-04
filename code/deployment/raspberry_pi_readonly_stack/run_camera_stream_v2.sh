#!/usr/bin/env bash
set -euo pipefail
stack_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
camera_source_dir="${NEXUS_CAMERA_SOURCE_DIR:-${HOME}/NEXUS/code/deployment/raspberry_pi_e016_self_localization/app}"
export PYTHONPATH="${camera_source_dir}:${stack_dir}:${PYTHONPATH:-}"
exec python3 "${stack_dir}/camera_stream_server.py" \
  --listen-host 0.0.0.0 --listen-port 14552 --width 1640 --height 1232 \
  --fps 10 --rotation 180 "$@"
