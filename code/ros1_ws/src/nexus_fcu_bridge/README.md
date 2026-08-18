# nexus_fcu_bridge（ROS1）

职责：复用幻思官方 `fcu_core`，通过 USB `/dev/ttyACM0` 或网络 `192.168.4.1:333` 接入 Mcontroller，并发布 UWB/光流/GNSS/IMU 状态。不要自写幻思协议解析。

官方基线：Ubuntu 20.04 + ROS Noetic + `catkin_make`，依赖 `quadrotor_msgs`。本包不承载 ROS2 算法；只把官方话题映射到 bridge 约定的 ROS2 话题，并把 ROS2 视觉位姿映射回 ROS1 `motion_001`。

目录约定：`src/` 节点，`config/` 参数，`launch/` 启动，`test/` 测试。消息映射和两端运行环境记录在 `docs/architecture/interfaces.md`。
