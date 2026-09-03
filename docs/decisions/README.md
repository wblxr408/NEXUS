# 架构决策记录（ADR）

用于记录会影响接口、依赖、算法、硬件或答辩结论的选择。不要把讨论过程埋在聊天记录里。

文件名：`YYYY-MM-DD_ADR-<三位编号>_<slug>.md`。

复制 `templates/adr.md`，填写状态（`proposed / accepted / superseded / rejected`）、背景、候选方案、决定、验证和影响。

## 当前已接受决定

- [ADR-001：采用双 ROS 分层与幻思官方 fcu_core 硬件基线](2026-08-18_ADR-001_fanci-hardware-ros-baseline.md)
- [ADR-002：固定外部目标定位范围与演示验收边界](2026-08-18_ADR-002_target-localization-scope.md)（对象范围与测量模型部分由 ADR-007 覆盖）
- [ADR-003：将液态神经网络限定为可选融合增强方案](2026-08-20_ADR-003_lnn_optional_fusion_enhancement.md)
- [ADR-004：首版视觉标记采用 AprilTag 36h11](2026-08-20_ADR-004_apriltag_first_version_marker.md)（排序已由 ADR-008 supersede；参数规则保留）
- [ADR-005：冻结硬件到货前接口与坐标契约 V1](2026-08-20_ADR-005_pre_hardware_interface_coordinate_contract.md)
- [ADR-006：目标观测契约 V2 与实时坐标链](2026-08-21_ADR-006_target_observation_contract_v2.md)
- [ADR-007：双对象迭代定位与机载视觉/UWB 测量模型](2026-08-24_ADR-007_dual_subject_iterative_localization.md)
- [ADR-008：无标签纯视觉优先与接口单一事实源](2026-08-25_ADR-008_markerless_visual_first_and_interface_authority.md)
- [ADR-009：无标记定位的深度观测通道与十目标对称群口径](2026-08-29_ADR-009_markerless_depth_channel_and_symmetry.md)
- [ADR-010：沙盘口径改为 CARLA RRD 地图与实机原始 UWB 测距可用性](2026-09-01_ADR-010_rrd_sandbox_and_hardware_uwb_range_availability.md)（参数化沙盘/sandbox-v29 场景口径降级为历史资料；ADR-009 第 5 项收窄为只对参数化沙盘成立；ADR-001 的「厂商可能不暴露原始 UWB 距离」警告已解除）
- [ADR-011：双对象定位、固定参考与观测可靠性学习](2026-09-03_ADR-011_robust_dual_localization.md)（最新用户设计与逐项实现边界；旧优先级变化见该记录）
- [ADR-012：相关目标米制状态融合与预测语义](2026-09-03_ADR-012_correlated_target_kinematic_fusion.md)（新消息直接消费、CI相关性处理、质量精确配对及非观测预测输出）
