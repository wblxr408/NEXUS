#!/usr/bin/env bash
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
host="${1:-192.168.1.143}"
user="${2:-pi}"
source_dir="${repo}/code/deployment/raspberry_pi_readonly_stack"
target_dir="/home/${user}/nexus_readonly_stack"

test -f "${source_dir}/uav_readonly_adapter_v2.py"
tar --exclude='__pycache__' --exclude='*.pyc' -C "${source_dir}" -czf - . |
  ssh "${user}@${host}" "mkdir -p '${target_dir}' && tar -C '${target_dir}' -xzf -"
ssh "${user}@${host}" \
  "chmod +x '${target_dir}'/*.py '${target_dir}'/*.sh && python3 -m py_compile '${target_dir}'/*.py"
