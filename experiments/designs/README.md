# 实验设计

每个设计说明研究问题、变量、采样计划、真值来源、指标和停止条件。建议命名 `E001_static_sensor_baseline.md`。

- [E001：静态目标定位基线与融合对比实验](E001_static_target_localization_baseline.md)：冻结测量模型后，比较官方 UWB、视觉与固定协方差融合的静态定位表现；不包含实测结论。
- [E002：UWB 定位复现与离线对比](E002_uwb_positioning_reproduction.md)：同一 synthetic trajectory 和 range observations 下比较五个 UWB 算法。
- [E003：UWB Gazebo 沙盘在线 smoke test](E003_uwb_sandbox_online_smoke.md)：记录 Gazebo、UWB adapter、router 和 Dashboard 的在线链路。
