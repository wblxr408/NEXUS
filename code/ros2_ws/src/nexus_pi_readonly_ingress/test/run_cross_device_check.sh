#!/usr/bin/env bash
set -eo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-49}"
export ROS_LOCALHOST_ONLY=0
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
host="${1:-192.168.1.143}"
config="${workspace}/src/nexus_pi_readonly_ingress/config/hardware_user_v01.yaml"
timeout 15 ros2 run nexus_pi_readonly_ingress telemetry_ingress_node.py --ros-args \
  -p hardware_config:="${config}" -p transport:=tcp_client \
  -p remote_host:="${host}" -p listen_port:=14551 \
  > /tmp/nexus_pi_cross_device_ingress.log 2>&1 &
node_pid=$!
cleanup() { kill "${node_pid}" 2>/dev/null || true; wait "${node_pid}" 2>/dev/null || true; }
trap cleanup EXIT
sleep 2
python3 "${workspace}/src/nexus_pi_readonly_ingress/test/cross_device_probe.py"
