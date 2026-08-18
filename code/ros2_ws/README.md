# ROS2 算法与应用工作空间

运行环境：Ubuntu 22.04 + ROS2 Humble + ament/colcon。

这里放项目自研的视觉定位、坐标转换、融合、评估、RViz2/Web 展示和主启动包。ROS1 的飞控数据通过 `ros1_bridge` 或轻量自定义桥接进入 ROS2；ROS2 视觉位姿按接口表回传 ROS1。

构建命令使用 `colcon build`，录包使用 `ros2 bag`。先实现 ROS2 单侧离线/回放链路，再接入 bridge。
