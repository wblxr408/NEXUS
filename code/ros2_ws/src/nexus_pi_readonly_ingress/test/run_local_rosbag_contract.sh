#!/usr/bin/env bash
set -eo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)"
bag="${1:-/tmp/nexus_pi_local_contract_bag}"
python3 "${repo}/code/ros2_ws/src/nexus_pi_readonly_ingress/test/synthetic_pi_streams.py" \
  --duration 20 >/tmp/nexus_synthetic_pi_streams.log 2>&1 &
server_pid=$!
cleanup() { kill "${server_pid}" 2>/dev/null || true; wait "${server_pid}" 2>/dev/null || true; }
trap cleanup EXIT
sleep 1
bash "${repo}/code/ros2_ws/src/nexus_pi_readonly_ingress/test/run_cross_device_rosbag_check.sh" \
  127.0.0.1 "${bag}"
