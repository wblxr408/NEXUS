# nexus_fusion_localization（ROS2）

职责：比较和组合可用的 UWB 目标观测、无人机平台位姿与视觉目标观测，统一输出 `/nexus/target/pose` 并评估目标定位误差。先实现可解释的单源基线和坐标求解，再按实验需要增加轻量 EKF/自适应修正；不替换飞控自身的位置闭环。

当前实时融合只接受 `map`、米制、显式有效且未过期的观测。双源时间差由 `max_pair_delta_ms` 控制；超限时发布最新单源并保留其来源模式，不标成融合。没有有效观测时发布带原因的无效状态；输入停止且缓存全部过期时由 watchdog 发布 `stale`，不发布零坐标或上一帧。

LNN（液态神经网络）仅可在正式单源基线和独立测试运行具备后，作为 ROS2 侧的可选时序质量评估/残差修正对比方案。它必须保留固定融合或最佳单源回退，不能直接替换飞控控制、把平台状态误作目标位置，或在无真值的样例上宣称改进。完整约束见 `docs/decisions/2026-08-20_ADR-003_lnn_optional_fusion_enhancement.md`。

## 2026-09-03 实现：目标异常值处理与质量 MLP

`fusion_mode=robust` 为当前默认：目标状态为 map 下的三维位置和速度，完整协方差用于创新马氏门控。正常更新、可疑降权、明显离群拒绝；测量协方差的学习放大在硬门控之后使用，不能放行已经被拒绝的观测。按 `(target_id, source_mode)` 拒绝重复或倒序采样，跨源迟到观测不回插状态。每个样本只融合一次。

`fusion_mode=information_baseline` 保留原信息加权基线及 `max_pair_delta_ms` 行为。鲁棒模式按每条观测的采样时间预测，不把不同时刻的位置直接平均。两种模式都只接收机外目标观测，机载 UWB 应进入平台定位链。

`reliability_mode` 有 `fixed`、`rule`、`learned`。规则模式按输入 confidence 放大协方差；学习模式必须指定经过训练的 `reliability_model` NPZ，不允许随机权重或静默替代模型。网络为 40→64→32→4：20 个质量特征及其可用性标志，隐层 ReLU、输出 sigmoid。NumPy 实现前向、反向和 Adam，无新增训练框架依赖。输出视觉/UWB/IMU可靠性与离群概率，将基础协方差放大 1–100 倍；IMU输出供平台分支后续使用，当前目标滤波不融合 IMU。

`prediction_horizon_ms` 内仅提供内部运动预测，`/nexus/target/pose` 不发布预测为有效观测。超过该时间后的恢复需要 `confirmation_hits` 次连续一致观测。门控阈值与运动参数是可配置的软件初始值，未完成硬件调参，不是实测保证。滤波状态确认不等同于视觉目标身份已确认。

新增诊断输出 `/nexus/target/fusion_status`，类型 `std_msgs/msg/String`，JSON schema_version=1，包含采样时间、target_id、source_mode、decision、reason、d2、计数、可靠性模式，以及接受时的协方差尺度和滤波轨迹状态。现有 target 消息未改变。

新增质量输入 `/nexus/observations/quality`，类型 `std_msgs/msg/String`，应在同一帧目标观测前到达：

```json
{"schema_version":1,"target_id":"target_1","source_mode":2,"sample_timestamp_ns":1234567890,"features":{"inlier_ratio":0.85,"reprojection_error_px":1.2,"tracked_points":40,"blur_metric":100.0}}
```

质量包必须精确匹配目标、来源和采样时间；过期/未来包拒绝。完整特征名与单位见 `reliability.py:FEATURE_NAMES`。缺失特征显式掩码；网络延迟、时间间隔和滤波速度从实际输入计算，不伪造 IMU 或几何质量。

训练 CSV 必须包含 `collection_group` 和 `vision_reliability,uwb_reliability,imu_reliability,outlier_probability` 四列监督标签，质量特征列可缺省。标签定义与真值依据通过 `--label-provenance` 登记。按整个批次/飞行片段划分训练/验证/测试；标准化仅用训练集，验证集选择训练轮次，测试集不参与选择。模型内保存划分、输入 SHA256 和指标，拒绝覆盖已有模型。

```bash
source code/ros2_ws/install/setup.bash
ros2 run nexus_fusion_localization train_reliability \
  --input /path/to/labelled_quality.csv --output /path/to/new_model.npz \
  --label-provenance RUN_ID_AND_LABEL_DEFINITION
```

验证：Humble 下构建 `colcon build --packages-up-to nexus_fusion_localization --symlink-install`，测试 `colcon test --packages-select nexus_fusion_localization`。DDS集成测试单独使用 localhost/domain 213，不连接硬件。当前训练测试为明确标注的合成数据；真实模型、平台滑窗和 SuperPoint 质量生产尚待完整集成，不能将当前目标滤波称为完整双对象融合系统。

## 平台IMU/UWB/视觉滑窗

`platform_localization_node`独立估计无人机的p/v/q/ba/bg，发布`/nexus/platform/odom`与`/nexus/platform/status`。它不把机载UWB当目标位置，也不发送飞行控制指令。数值内核与ROS运行接口见`docs/architecture/2026-09-03_design_platform_fusion_interfaces.md`。

```bash
source code/ros2_ws/install/setup.bash
ros2 launch nexus_fusion_localization platform_localization.launch.py \
  calibration_file:=/path/to/measured_platform.yaml
ros2 launch nexus_vision_localization fixed_reference.launch.py \
  calibration_file:=/path/to/measured_fixed_tags.yaml
```

启动前必须提供本场地实测标定。平台YAML包含schema_version=1、calibration_id、calibration_status=measured、map_frame=map、body_frame=base_link；imu字段包含frame_id、rotation_body_imu、at_body_origin_or_compensated与全部noise字段（accel_density、gyro_density、accel_bias_walk、gyro_bias_walk、maximum_gap_s）；uwb字段包含tag_body_m和按ID索引的anchors_map_m；还需initial_velocity_sigma_mps、initial_accel_bias_sigma_mps2、initial_gyro_bias_sigma_rps。可选window字段覆盖PlatformConfig参数。所有距离/速度/加速度均使用SI单位。

固定Tag标定包含schema_version、calibration_id、calibration_status、reference_frame=map、camera_frame、tag_family、image_rectified、transform_body_camera、extrinsic_covariance、reference_covariance、pixel_sigma_px，以及tags列表的id/size_m/transform_map_tag。相机内参与图像几何来自对应CameraInfo；Tag检测来自apriltag_ros。测试样例在`test_platform_node.py`和`test_fixed_reference_node.py`中明确标为synthetic_test并要求显式opt-in，不提供猜测的实机标定文件。

默认六状态窗口使用IMU预积分、外部因子鲁棒IRLS及真正Schur边缘化。基础协方差先用于创新门控，再施加1–100倍可靠性缩放。缺IMU区间、异常位姿、过期观测和求解失败会产生诊断；超过预测时限失效。原采样时间在窗口内可插入，不将网络到达时间冒充采样时间。

平台数值测试与解析导数校验通过，ROS测试使用localhost/domain 214，固定参考测试domain 215。当前是独立模块与合成消息验证，SuperPoint完整前端、端到端延迟及实机精度仍待完成；最新逐项状态见`docs/2026-09-03_checklist_dual_localization_implementation.md`。

## 新米制目标状态的相关融合

`target_kinematic_fusion_node`直接订阅`/nexus/vision/target_kinematics`并发布`/nexus/target/kinematics`，保留p/v/map角的9×9协方差与未知分量NaN。重叠多帧输出使用CI处理未知相关性，避免按独立测量重复缩小协方差；本节点不接入机载UWB目标位置，不订阅自身输出。

支持fixed/rule/learned权重、按来源/参考点/原采样时间精确配对质量、有界乱序等待、硬创新拒绝、参考点与身份歧义恢复、限时回滚。短时预测的valid与三个observed均为false，reason=prediction_only，last_valid_sample_timestamp_ns不刷新；预测过期或身份不确定时输出NaN。界面不得把有限预测坐标当作新观测。

独立启动：`ros2 launch nexus_fusion_localization target_kinematic_fusion.launch.py`；需要学习权重时传入reliability_mode=learned及真实训练的reliability_model路径。`static_visual.launch.py`已经默认包含该节点，不要重复启动。详细配置、协方差和消息语义见`docs/architecture/2026-09-03_design_target_kinematic_fusion_interfaces.md`，测试用localhost/domain 228。现有旧消息节点保持原路径；两条路径不能因target_id相同就丢弃position_reference后混合。
