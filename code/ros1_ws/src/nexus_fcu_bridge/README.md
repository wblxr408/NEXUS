# nexus_fcu_bridge（ROS1）

职责：复用幻思官方 `fcu_core`，通过 USB `/dev/ttyACM0` 或网络 `192.168.4.1:333` 接入 Mcontroller，并发布飞行平台状态和厂商实际开放的 UWB/光流/GNSS/IMU 数据。不要自写幻思协议解析，也不要把无人机位姿直接标成外部目标位姿。

官方基线：Ubuntu 20.04 + ROS Noetic + `catkin_make`，依赖 `quadrotor_msgs`。本包不承载 ROS2 算法；只把目标定位所需的官方话题、采样时间、frame 和单位映射到跨 ROS 信息通道。ROS2 结果到 ROS1 `motion_001` 的回传属于可选控制扩展，不是首版验收条件。

目录约定：`src/` 节点，`config/` 参数，`launch/` 启动，`test/` 测试。消息映射和两端运行环境记录在 `docs/architecture/interfaces.md`。
