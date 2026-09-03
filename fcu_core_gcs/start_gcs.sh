#!/bin/bash
# 一键启动 fcu_core（gcs 地面站端）
cd "$(dirname "$0")"
source install/setup.bash
ros2 launch fcu_core fcu_core_launch.py mode:=cli
