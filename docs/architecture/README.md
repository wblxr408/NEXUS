# 系统架构与接口

## 分层

1. `nexus_uwb_driver`：串口/网络接入、原始测距和设备状态。
2. `nexus_vision_localization`：多相机采集、AprilTag/ArUco 检测、三角测量输出。
3. `nexus_coord_transform`：传感器、机体、沙盘世界坐标系与 `tf2`。
4. `nexus_fusion_localization`：EKF/UKF 基线、自适应权重和遮挡/NLOS 降级。
5. `nexus_viz_dashboard`：RViz2、rosbridge 和 Web Dashboard。
6. `nexus_bringup`：参数、launch、录包与一键启动。

## 接口原则

- Topic、frame、单位和时间戳先写入接口表，再写节点代码。
- 所有位置默认使用米、时间使用 ROS 时间；禁止用到达时间替代采样时间而不作说明。
- 每个节点 README 必须写输入、输出、参数、故障行为和验证命令。

详细坐标约定见 `coordinate-frames.md`；接口表见 `interfaces.md`。
