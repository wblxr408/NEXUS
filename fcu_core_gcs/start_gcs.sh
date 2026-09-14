#!/usr/bin/env bash
# 一键启动 fcu_core（gcs 地面站端）
set -euo pipefail
gcs_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "${gcs_dir}/.." && pwd)"
camera_workspace_setup="${repo_dir}/code/ros2_ws/install/setup.bash"

cd "${gcs_dir}"
set +u
source /opt/ros/humble/setup.bash
if [[ ! -f "${camera_workspace_setup}" ]]; then
  printf '相机接收工作空间尚未构建：%s\n' "${camera_workspace_setup}" >&2
  printf '请先构建 nexus_pi_readonly_ingress。\n' >&2
  exit 1
fi
source "${camera_workspace_setup}"
source install/setup.bash
set -u
export ROS_DOMAIN_ID=20

camera_pid=""
viewer_pid=""
cleanup() {
  [[ -z "${viewer_pid}" ]] || kill "${viewer_pid}" 2>/dev/null || true
  [[ -z "${camera_pid}" ]] || kill "${camera_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 run nexus_pi_readonly_ingress camera_file_node.py --ros-args \
  -p remote_host:=192.168.1.143 \
  -p remote_port:=14552 \
  -p clock_sync:=true \
  -p activation_command_topic:=/command \
  -p start_command:=3 \
  -p stop_commands:="[2, 4]" &
camera_pid=$!

camera_ready=0
for _ in $(seq 1 50); do
  if ! kill -0 "${camera_pid}" 2>/dev/null; then
    wait "${camera_pid}" || true
    printf '相机接收器启动失败，GCS 未启动。\n' >&2
    exit 1
  fi
  if ros2 node list 2>/dev/null | grep -q '^/nexus_pi_camera_file_ingress$'; then
    camera_ready=1
    break
  fi
  sleep 0.1
done
if [[ "${camera_ready}" != "1" ]]; then
  printf '相机接收器在 5 秒内未就绪，GCS 未启动。\n' >&2
  exit 1
fi

if [[ "${NEXUS_CAMERA_VIEW:-1}" != "0" ]]; then
  ros2 run rqt_image_view rqt_image_view /nexus/camera/imx219/image_raw &
  viewer_pid=$!
fi

printf '相机接收器已就绪：按 t 后开始回传，按 l 或 d 后停止。\n'
ros2 launch fcu_core fcu_core_launch.py mode:=cli
