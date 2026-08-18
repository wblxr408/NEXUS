# ADR-001：采用双 ROS 分层与幻思官方 fcu_core 硬件基线

- 日期：2026-08-18
- 状态：accepted
- 负责人：项目组
- 关联任务：ROS 接入、UWB 基线、视觉回传

## 背景

实际硬件为 FanciSwarm/Mcontroller V7 和 FanciSwarm UWB 四基站。幻思公开的 `fcu_core` 是 Ubuntu 20.04 + ROS Noetic + catkin 工程，支持 USB/TCP、MAVLink、飞控状态读取和视觉位姿回传。

## 决定

采用双 ROS 分层：Ubuntu 20.04 + ROS1 Noetic + 官方 `fcu_core` 作为硬件兼容层；Ubuntu 22.04 + ROS2 Humble 作为项目算法、坐标、融合、评估和可视化主线。两侧通过 `ros1_bridge` 或轻量自定义 bridge 连接。飞控端继续运行 Mcontroller 自带 FreeRTOS、UWB/光流/测距和位置控制闭环。

## 拒绝的方案

- Nooploop 驱动：硬件不匹配。
- 把 ROS1 官方工程强行迁移到 ROS2：会偏离厂商公开基线；ROS2 自研模块不需要等待迁移完成。
- 立即重写完整 EKF：会绕开已经能飞的厂商闭环，先用真实回传效果证明必要性。

## 影响

- ROS1 工作空间使用 `catkin_make`、`roslaunch`、`rosbag` 和 `tf`；ROS2 工作空间使用 `colcon build`、`ros2 launch`、`ros2 bag` 和 `tf2`。
- UWB 算法对比以官方位置输出为必做基线；原始测距算法只有在厂商开放数据后才启用。
- 视觉输入和多相机阵列属于条件性模块，必须先通过硬件接口验收。

## 验证条件

以 `docs/architecture/hardware-integration.md` 的首次联调验收清单为准；每次结论登记到 `docs/records/evidence-index.md`。
