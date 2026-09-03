#!/bin/bash

# 自动构建脚本 for fcu_core_gcs (地面站)
# 使用说明：
# 方式1：构建项目（推荐）
#   ./build.sh
#   然后手动加载环境：source install/setup.bash
#
# 方式2：构建并加载环境
#   source build.sh
#   环境变量会在当前终端生效

set -e  # 遇到错误立即退出

echo "========================================="
echo "开始清理旧的构建文件..."
echo "========================================="

# 清理旧的构建文件
rm -rf build install log

echo "清理完成！"
echo ""

echo "========================================="
echo "编译所有包（fcu_core, rviz2_custom_panel）..."
echo "========================================="

# 编译所有包
colcon build

echo ""
echo "========================================="
echo "加载ROS2环境..."
echo "========================================="

# 加载ROS2环境
source install/setup.bash

echo "环境加载完成！"
echo ""
echo "========================================="
echo "构建完成！"
echo "========================================="
echo ""
echo "提示："
echo "  如果使用 './build.sh' 执行，需要运行以下命令加载环境："
echo "    source install/setup.bash"
echo "  如果使用 'source build.sh' 执行，环境已加载，可以直接使用"
echo ""
echo "启动方式："
echo "  RViz面板控制：  ros2 launch fcu_core fcu_core_launch.py"
echo "  命令行控制：    ros2 launch fcu_core fcu_core_launch.py mode:=cli"
