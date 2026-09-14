#!/usr/bin/env bash
set -eo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../ros2_ws" && pwd)"
source /opt/ros/humble/setup.bash
source "${NEXUS_ROS_INSTALL:-${workspace}/install}/setup.bash"
set -u
output="${1:?usage: record_pi_readonly_rosbag.sh OUTPUT_DIRECTORY}"
exec ros2 bag record -o "${output}" \
  /nexus/fcu/imu \
  /nexus/uwb/ranges \
  /nexus/pi/telemetry_health \
  /nexus/camera/imx219/image_raw \
  /nexus/camera/imx219/image_raw/compressed \
  /nexus/camera/imx219/camera_info \
  /nexus/camera/imx219/metadata \
  /nexus/pi/camera_health
