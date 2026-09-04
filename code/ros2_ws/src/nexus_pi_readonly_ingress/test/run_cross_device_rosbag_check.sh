#!/usr/bin/env bash
set -eo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-51}"
export ROS_LOCALHOST_ONLY=0
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
host="${1:-192.168.1.143}"
bag="${2:-/tmp/nexus_pi_cross_device_bag}"
config="${workspace}/src/nexus_pi_readonly_ingress/config/hardware_user_v01.yaml"
if [[ -e "${bag}" ]]; then
  echo "refusing to overwrite existing bag: ${bag}" >&2
  exit 2
fi
timeout 15 ros2 launch nexus_pi_readonly_ingress readonly_ingress.launch.py \
  hardware_config:="${config}" remote_host:="${host}" \
  > /tmp/nexus_pi_cross_device_launch.log 2>&1 &
launch_pid=$!
cleanup() { kill "${launch_pid}" 2>/dev/null || true; wait "${launch_pid}" 2>/dev/null || true; }
trap cleanup EXIT
sleep 3
timeout --signal=INT 7 "${workspace}/../tools/record_pi_readonly_rosbag.sh" "${bag}" || test $? -eq 124
python3 - "${bag}" <<'PY'
import json
import sys
import rosbag2_py

bag = sys.argv[1]
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("", ""))
counts = {}
while reader.has_next():
    topic, _, _ = reader.read_next()
    counts[topic] = counts.get(topic, 0) + 1
required = [
    "/nexus/fcu/imu", "/nexus/uwb/ranges", "/nexus/pi/telemetry_health",
    "/nexus/camera/imx219/image_raw", "/nexus/camera/imx219/image_raw/compressed",
    "/nexus/camera/imx219/camera_info", "/nexus/camera/imx219/metadata",
    "/nexus/pi/camera_health",
]
print(json.dumps(counts, sort_keys=True))
missing = [topic for topic in required if counts.get(topic, 0) == 0]
if missing:
    raise SystemExit("missing recorded topics: " + ", ".join(missing))
PY
