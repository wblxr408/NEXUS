# 系统架构与接口（双 ROS / FanciSwarm 硬件适配版）

## 分层

1. **ROS1 硬件层（Ubuntu 20.04 + Noetic）**：`fcu_core` / `nexus_fcu_bridge` 连接 Mcontroller V7，读取 UWB/光流/GNSS/IMU，并接收视觉回传。
2. **ROS1-ROS2 bridge**：只做消息、时间戳、坐标和单位的明确映射，不把两套 ROS 混装进同一工作空间。
3. **ROS2 算法层（Ubuntu 22.04 + Humble）**：`nexus_vision_localization`、`nexus_coord_transform`、`nexus_fusion_localization` 完成视觉、`tf2`、融合与评估。
4. **ROS2 应用层**：`nexus_viz_dashboard` 与 `nexus_bringup` 管理 RViz2、rosbridge、`ros2 bag` 和演示启动。

## 接口原则

- Topic、frame、单位和时间戳先写入接口表，再写节点代码。
- 所有位置默认使用米、时间使用 ROS 时间；禁止用到达时间替代采样时间而不作说明。
- 每个节点 README 必须写输入、输出、参数、故障行为和验证命令。

Mcontroller 端保持官方 FreeRTOS/飞控闭环；ROS1 负责硬件兼容，ROS2 负责项目主线算法和展示。硬件接口基准见 `hardware-integration.md`；详细坐标约定见 `coordinate-frames.md`；接口表见 `interfaces.md`。当前 bridge 话题名是项目草案，必须先回放验证再固化。
