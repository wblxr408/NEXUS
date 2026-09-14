#!/usr/bin/env bash
set -euo pipefail
stack_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
camera_source_dir="${NEXUS_CAMERA_SOURCE_DIR:-${HOME}/NEXUS/code/deployment/raspberry_pi_e016_self_localization/app}"
export PYTHONPATH="${camera_source_dir}:${PYTHONPATH:-}"
duration="${1:-30}"
arm_timeout="${NEXUS_ARM_TIMEOUT_SECONDS:-120}"
run_root="${NEXUS_RUN_ROOT:-${HOME}/nexus_readonly_runs}"
run_id="$(date -u +%Y%m%dT%H%M%SZ)_pi_readonly_params_v02"
run_dir="${run_root}/${run_id}"
mkdir -p "${run_dir}/camera"
cp "${stack_dir}/config/hardware_user_v02.yaml" "${run_dir}/hardware.yaml"
camera_pid=""
telemetry_pid=""
cleanup() {
  [[ -n "${camera_pid}" ]] && kill "${camera_pid}" 2>/dev/null || true
  [[ -n "${telemetry_pid}" ]] && kill "${telemetry_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
"${stack_dir}/run_telemetry_tag2_domain0.sh" --output "${run_dir}/telemetry.jsonl" \
  >"${run_dir}/telemetry.log" 2>&1 &
telemetry_pid=$!
printf 'Waiting for online=true and armed=true telemetry (timeout: %s seconds).\n' "${arm_timeout}"
if ! python3 "${stack_dir}/wait_for_armed.py" \
  --telemetry-jsonl "${run_dir}/telemetry.jsonl" \
  --timeout-seconds "${arm_timeout}" \
  --event-output "${run_dir}/camera_arm_trigger.json" >"${run_dir}/arm_trigger.log" 2>&1; then
  printf 'No armed telemetry received; camera capture was not started. See %s\n' "${run_dir}/arm_trigger.log" >&2
  exit 8
fi
python3 "${stack_dir}/capture_live_camera_v2.py" --output-dir "${run_dir}/camera" \
  --width 1640 --height 1232 --fps 30 --rotation 180 --duration-seconds "${duration}" \
  >"${run_dir}/camera.log" 2>&1 &
camera_pid=$!
wait "${camera_pid}"
camera_pid=""
kill "${telemetry_pid}" 2>/dev/null || true
wait "${telemetry_pid}" 2>/dev/null || true
telemetry_pid=""
python3 "${stack_dir}/readonly_health_check.py" \
  --telemetry-jsonl "${run_dir}/telemetry.jsonl" \
  --camera-metadata "${run_dir}/camera/latest.json" \
  --max-age-seconds 10 >"${run_dir}/health.json" || true
printf '%s\n' "${run_dir}"
