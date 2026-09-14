#!/usr/bin/env bash
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)"
install_prefix="${NEXUS_ROS_INSTALL:-${repo}/code/ros2_ws/install}"
python3 "${repo}/code/ros2_ws/src/nexus_pi_readonly_ingress/test/synthetic_pi_streams.py" --duration 15 \
  > /tmp/nexus_synthetic_pi_content_contract.log 2>&1 &
server_pid=$!
telemetry_pid=
camera_pid=
cleanup() {
  [[ -n "${telemetry_pid}" ]] && kill "${telemetry_pid}" 2>/dev/null || true
  [[ -n "${camera_pid}" ]] && kill "${camera_pid}" 2>/dev/null || true
  kill "${server_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
sleep 1
export NEXUS_ROS_INSTALL="${install_prefix}"
bash "${repo}/code/ros2_ws/src/nexus_pi_readonly_ingress/test/run_cross_device_check.sh" 127.0.0.1 &
telemetry_pid=$!
bash "${repo}/code/ros2_ws/src/nexus_pi_readonly_ingress/test/run_camera_cross_device_check.sh" 127.0.0.1 &
camera_pid=$!
telemetry_rc=0
camera_rc=0
wait "${telemetry_pid}" || telemetry_rc=$?
wait "${camera_pid}" || camera_rc=$?
printf 'TELEMETRY_RC=%s CAMERA_RC=%s\n' "${telemetry_rc}" "${camera_rc}"
[[ "${telemetry_rc}" -eq 0 && "${camera_rc}" -eq 0 ]]
