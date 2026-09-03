# ROS2 算法与应用工作空间

运行环境：Ubuntu 22.04 + ROS2 Humble + ament/colcon。

这里放项目自研的外部目标视觉定位、坐标转换、算法比较/融合、评估、RViz2/Web 展示和主启动包。ROS1 的飞行平台状态和可用 UWB 数据通过显式信息通道进入 ROS2；ROS2 统一输出 `/nexus/target/pose`。只有目标跟踪控制需要时才按接口表回传 ROS1。

构建命令使用 `colcon build`，录包使用 `ros2 bag`。先实现 ROS2 单侧离线/回放链路，再接入 bridge。

## Humble 构建基线

先安装本工作空间的 Python/ament 测试依赖：

```bash
sudo apt-get update
sudo apt-get install python3-ament-package python3-pytest
```

每个新终端进入工作空间后，先加载项目环境：

```bash
source scripts/activate_humble.sh
```

该脚本会加载 ROS 2 Humble，并在 WSL 中把 `TMPDIR` 固定到 Linux 的
`/tmp`。这避免继承 Windows 挂载路径的 `TMP`/`TEMP` 时，`pytest` 在
ament CMake 配置检查中失败、从而使 Python 测试未被登记的问题。

无标签纯视觉是当前算法复现主线；工作空间中的 `apriltag_ros` 仅作为延后 AprilTag 36h11 工程基线保留。需要复现该基线时，在 Ubuntu 22.04 + ROS2 Humble 主机上安装其系统依赖：

```bash
sudo apt-get install ros-humble-apriltag ros-humble-apriltag-msgs
```

随后在本目录执行完整检查：

```bash
source scripts/activate_humble.sh
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

`code/third_party/rosbridge_suite` 是浏览器数据出口的上游参考，故意不位于本工作空间；其当前固定提交不是 Humble 的默认构建输入。实际接 Dashboard 时使用经 Humble 验证的系统包或另行登记兼容提交。
