# ROS1 硬件兼容工作空间

运行环境：Ubuntu 20.04 + ROS Noetic + catkin。

这里仅放幻思官方 `fcu_core` 的外部源码登记和 `nexus_fcu_bridge` 适配。它负责连接 Mcontroller V7，读取飞行平台状态和厂商实际开放的 UWB/光流/GNSS/IMU 数据，并通过显式信息通道提供给 ROS2。只有演示需要目标跟踪控制时，才接收 ROS2 结果并通过 `motion_001` 回传。构建命令使用 `catkin_make`，录包使用 `rosbag`。

`fcu_core_external/` 只登记厂商源码来源和版本，不复制未经确认的第三方协议实现。
