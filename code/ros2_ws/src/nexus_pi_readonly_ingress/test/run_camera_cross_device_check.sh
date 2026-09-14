#!/usr/bin/env bash
set -eo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source /opt/ros/humble/setup.bash
source "${NEXUS_ROS_INSTALL:-${workspace}/install}/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-50}"
export ROS_LOCALHOST_ONLY=0
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
host="${1:-192.168.1.143}"
timeout 18 ros2 run nexus_pi_readonly_ingress camera_file_node.py --ros-args \
  -p transport:=tcp_client -p remote_host:="${host}" -p remote_port:=14552 \
  > /tmp/nexus_pi_camera_cross_device_ingress.log 2>&1 &
node_pid=$!
cleanup() { kill "${node_pid}" 2>/dev/null || true; wait "${node_pid}" 2>/dev/null || true; }
trap cleanup EXIT
sleep 2
python3 "${workspace}/src/nexus_pi_readonly_ingress/test/camera_cross_device_probe.py"
