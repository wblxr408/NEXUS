# 系统架构与接口（双 ROS / FanciSwarm 硬件适配版）

项目对象以 [ADR-002](../decisions/2026-08-18_ADR-002_target-localization-scope.md) 为准：被定位对象是无人机之外的目标。无人机自身位姿、目标相对观测和目标世界位姿是三种不同语义，任何 topic、frame 或图表都不得混用。

## 分层

1. **ROS1 硬件层（Ubuntu 20.04 + Noetic）**：`fcu_core` / `nexus_fcu_bridge` 连接 Mcontroller V7，读取飞行平台状态和厂商实际开放的 UWB/光流/GNSS/IMU 数据。
2. **ROS1-ROS2 信息通道**：把目标定位所需的消息、采样时间、坐标和单位显式映射到 ROS2；具体采用标准 bridge、专用桥接环境或轻量网络通道仍待验证。
3. **ROS2 算法层（Ubuntu 22.04 + Humble）**：`nexus_vision_localization`、`nexus_coord_transform`、`nexus_fusion_localization` 完成目标观测、`tf2`、目标坐标求解、算法比较与评估。
4. **ROS2 应用层**：`nexus_viz_dashboard` 与 `nexus_bringup` 管理 RViz2、rosbridge、`ros2 bag`、图表和演示视频输入。

## 接口原则

- Topic、frame、单位和时间戳先写入接口表，再写节点代码。
- 所有位置默认使用米、时间使用 ROS 时间；禁止用到达时间替代采样时间而不作说明。
- 每个节点 README 必须写输入、输出、参数、故障行为和验证命令。

Mcontroller 端保持官方 FreeRTOS/飞控闭环；ROS1 负责硬件兼容，ROS2 负责外部目标定位和展示。ROS2→ROS1 控制回传只在演示明确需要目标跟踪时启用。硬件接口基准见 `hardware-integration.md`；详细坐标约定见 `coordinate-frames.md`；接口表见 `interfaces.md`。当前信息通道和话题名都是项目草案，必须先回放验证再固化。

演示网页的信息层级、实时/回放状态和相机预览位置见 [Dashboard 信息架构设计](2026-08-19_design_demo_dashboard_information_architecture.md)。
