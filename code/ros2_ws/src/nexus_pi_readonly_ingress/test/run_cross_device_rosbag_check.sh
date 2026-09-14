#!/usr/bin/env bash
set -eo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source /opt/ros/humble/setup.bash
source "${NEXUS_ROS_INSTALL:-${workspace}/install}/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-51}"
export ROS_LOCALHOST_ONLY=0
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
host="${1:-192.168.1.143}"
bag="${2:-/tmp/nexus_pi_cross_device_bag}"
launch_timeout_seconds="${NEXUS_LAUNCH_TIMEOUT_SECONDS:-20}"
warmup_seconds="${NEXUS_WARMUP_SECONDS:-3}"
record_seconds="${NEXUS_RECORD_SECONDS:-7}"
config="${workspace}/src/nexus_pi_readonly_ingress/config/hardware_user_v02.yaml"
if [[ -e "${bag}" ]]; then
  echo "refusing to overwrite existing bag: ${bag}" >&2
  exit 2
fi
timeout "${launch_timeout_seconds}" ros2 launch nexus_pi_readonly_ingress readonly_ingress_v02.launch.py \
  hardware_config:="${config}" remote_host:="${host}" \
  > /tmp/nexus_pi_cross_device_launch.log 2>&1 &
launch_pid=$!
cleanup() { kill "${launch_pid}" 2>/dev/null || true; wait "${launch_pid}" 2>/dev/null || true; }
trap cleanup EXIT
sleep "${warmup_seconds}"
timeout --signal=INT "${record_seconds}" "${workspace}/../tools/record_pi_readonly_rosbag.sh" "${bag}" || test $? -eq 124
python3 - "${bag}" <<'PY'
import json
import math
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String

bag = sys.argv[1]
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("", ""))
counts = {}
json_samples = {}
image_samples = []
imu_samples = []
while reader.has_next():
    topic, serialized, _ = reader.read_next()
    counts[topic] = counts.get(topic, 0) + 1
    if topic in {
        "/nexus/uwb/ranges", "/nexus/pi/telemetry_health",
        "/nexus/camera/imx219/metadata", "/nexus/pi/camera_health",
    }:
        json_samples.setdefault(topic, []).append(
            json.loads(deserialize_message(serialized, String).data))
    elif topic == "/nexus/camera/imx219/image_raw":
        message = deserialize_message(serialized, Image)
        image_samples.append({"width": message.width, "height": message.height,
                              "encoding": message.encoding})
    elif topic == "/nexus/fcu/imu":
        message = deserialize_message(serialized, Imu)
        imu_samples.append(message.linear_acceleration.z)

required = [
    "/nexus/fcu/imu", "/nexus/uwb/ranges", "/nexus/pi/telemetry_health",
    "/nexus/camera/imx219/image_raw", "/nexus/camera/imx219/image_raw/compressed",
    "/nexus/camera/imx219/camera_info", "/nexus/camera/imx219/metadata",
    "/nexus/pi/camera_health",
]
missing = [topic for topic in required if counts.get(topic, 0) == 0]
if missing:
    raise SystemExit("missing recorded topics: " + ", ".join(missing))

uwb = json_samples["/nexus/uwb/ranges"]
metadata = json_samples["/nexus/camera/imx219/metadata"]
camera_health = json_samples["/nexus/pi/camera_health"]
telemetry_health = json_samples["/nexus/pi/telemetry_health"]
checks = {
    "tag2_four_anchors": any(
        item.get("tag_id") == 2 and item.get("anchor_ids") == ["1", "2", "3", "4"]
        for item in uwb),
    "imu_finite": any(math.isfinite(value) for value in imu_samples),
    "camera_1640x1232_rgb8": any(
        item == {"width": 1640, "height": 1232, "encoding": "rgb8"}
        for item in image_samples),
    "camera_timestamp_sequence_latency": any(
        item.get("sensor_timestamp_ns") is not None
        and isinstance(item.get("frame_sequence"), int)
        and item.get("transport_latency_ms") is not None
        and math.isfinite(float(item["transport_latency_ms"]))
        for item in metadata),
    "camera_health_valid": any(
        item.get("valid") is True
        and item.get("estimated_missing_frames") is not None
        and item.get("last_receive_age_ms") is not None
        for item in camera_health),
    "telemetry_health_valid": any(item.get("valid") is True for item in telemetry_health),
}
print(json.dumps({"counts": counts, "checks": checks}, sort_keys=True))
failed = [name for name, passed in checks.items() if not passed]
if failed:
    raise SystemExit("recorded content checks failed: " + ", ".join(failed))
PY
