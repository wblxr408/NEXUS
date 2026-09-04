# RRD 训练与 IMU 派生数据登记

- 原压缩包 SHA256：`7759de118d35e834e3a8a03107bd5f927d55bfebc90ba5c15d9abdd27c17d052`。
- 来源类型：CARLA RRD 仿真；500 帧，20 Hz 图像/平台位姿，三类静态目标，合成 UWB。
- 本地审计展开目录：`data/interim/2026-09-04_rrd_training_audit_v01/`（Git 忽略）。
- 检测训练外部目录：`C:/Users/wblxr/.codex/visualizations/2026/09/03/01a065c8-bb9e-7752-93d1-4ef1f2ba16d1/rrd_detector_training_v01/`。
- IMU 输出：`data/processed/2026-09-04_rrd_imu_ideal_v01/` 与 `data/processed/2026-09-04_rrd_imu_assumed_v01/`（Git 忽略）；各自 `manifest.json` 保存源位姿及数据文件 SHA256。
- 坐标和单位：CARLA 左手世界 y 轴翻转为右手 `map`；虚拟 IMU 在机体原点；加速度 `m/s²`，角速度 `rad/s`，时间为仿真秒与对应整数纳秒。
- 已确认硬件依据：188.9 Hz、厂商线加速度 `mm/s²`、角速度 `mrad/s`；单条静止陀螺读数仅作为粗 bias 场景值。噪声密度、加速度偏置、bias 稳定性和安装旋转仍未标定。

完整过程、指标和限制见 [`2026-09-04_E023_rrd_detector_finetune`](../../experiments/runs/2026-09-04_E023_rrd_detector_finetune/run.md)。
