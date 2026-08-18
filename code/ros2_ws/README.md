# ROS2 算法与应用工作空间

运行环境：Ubuntu 22.04 + ROS2 Humble + ament/colcon。

这里放项目自研的外部目标视觉定位、坐标转换、算法比较/融合、评估、RViz2/Web 展示和主启动包。ROS1 的飞行平台状态和可用 UWB 数据通过显式信息通道进入 ROS2；ROS2 统一输出 `/nexus/target/pose`。只有目标跟踪控制需要时才按接口表回传 ROS1。

构建命令使用 `colcon build`，录包使用 `ros2 bag`。先实现 ROS2 单侧离线/回放链路，再接入 bridge。
