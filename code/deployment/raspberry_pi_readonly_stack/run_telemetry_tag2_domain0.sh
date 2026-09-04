#!/usr/bin/env bash
set -euo pipefail
stack_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID=0
if [[ -f "${HOME}/mavlink_env/bin/activate" ]]; then
  source "${HOME}/mavlink_env/bin/activate"
fi
export PYTHONPATH="${stack_dir}:${PYTHONPATH:-}"
exec python3 "${stack_dir}/uav_readonly_adapter_v2.py" --uwb-tag-id 2 "$@"
