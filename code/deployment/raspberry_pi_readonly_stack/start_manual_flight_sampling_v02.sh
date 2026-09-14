#!/usr/bin/env bash
set -euo pipefail

stack_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
duration="${1:-60}"

if ! [[ "${duration}" =~ ^[1-9][0-9]*$ ]]; then
  printf '用法: %s [采样秒数，正整数]\n' "${0##*/}" >&2
  exit 2
fi

if pgrep -af '[s]ynthetic_telemetry_test_server.py' >/dev/null; then
  printf '拒绝启动：检测到模拟遥测进程。请先停止模拟源，避免污染真实飞行数据。\n' >&2
  exit 3
fi

if pgrep -af '[r]ecord_readonly_params_v02.sh|[c]apture_live_camera_v2.py|[u]av_readonly_adapter_v2.py' >/dev/null; then
  printf '拒绝启动：检测到另一组只读采样进程，避免重复录制或占用相机。\n' >&2
  exit 4
fi

if pgrep -af '[c]amera_stream_server.py' >/dev/null; then
  printf '拒绝启动：实时相机流正在占用 IMX219。请先停止该只读相机流，再重试。\n' >&2
  exit 5
fi

if [[ ! -x "${stack_dir}/record_readonly_params_v02.sh" ]]; then
  printf '无法启动：缺少可执行的 record_readonly_params_v02.sh。\n' >&2
  exit 6
fi

run_root="${NEXUS_RUN_ROOT:-${HOME}/nexus_readonly_runs}"
mkdir -p "${run_root}"
if [[ ! -w "${run_root}" ]]; then
  printf '无法启动：采样目录不可写：%s\n' "${run_root}" >&2
  exit 7
fi

printf '即将开始 %s 秒只读采样。\n' "${duration}"
printf '本脚本只采集 IMX219、IMU 和 UWB，不解锁、不起飞、不发送速度或高度指令。\n'
printf '采样开始时间（UTC）：%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"

exec "${stack_dir}/record_readonly_params_v02.sh" "${duration}"
