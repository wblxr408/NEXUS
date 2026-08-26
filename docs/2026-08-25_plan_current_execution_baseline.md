# 当前执行基线：无标签纯视觉优先

- 日期：2026-08-25
- 状态：current
- 决策依据：`docs/decisions/2026-08-25_ADR-008_markerless_visual_first_and_interface_authority.md`

本文件是硬件到货前和算法复现阶段的唯一执行入口。规划书和模块图保留为范围及候选参考；实验运行和代码是结果事实源。

## 1. 目标与边界

- 同时定位无人机自身和无人机之外的目标。UWB/飞控测量 `base_link`，机载视觉测量 `target_link`，两者在统一 `map` 中迭代。
- 首要算法复现是无标签纯视觉目标定位，不依赖目标上安装的标签。
- AprilTag 36h11 仅是延后工程基线；有时间再做，不能代替无标签算法验收。
- ROS1 Noetic + 官方 `fcu_core` 只负责硬件兼容；ROS2 Humble 负责视觉、坐标变换、评估、融合和演示。
- 规划中的精度数字仅为目标；只有 `experiments/runs/` 有完整证据时才能报告实测精度。

## 2. 复现路线

### P0：无标签纯视觉

先根据目标可观测性选一条可复现路线，不同时铺开多个网络：

| 条件 | 首选复现 | 必需输入 | 失败时记录 |
|---|---|---|---|
| 有 CAD/尺寸和 RGB（可选深度） | FoundationPose 或同类现成 6D 方法 | 图像、内参、目标模型；深度则需深度内参/对齐 | 模型许可、GPU、目标域差、尺度/姿态失锁 |
| 无 CAD、目标固定且有稳定纹理 | 固定沙盘建图 + 局部特征匹配 + PnP | 图像、内参、固定地图、验证点 | 纹理不足、视角变化、匹配率、重投影误差 |
| 无几何、无深度且低纹理 | 暂停算法替换，先补可观测性 | 目标几何、深度、双目或可用纹理之一 | 这是数据条件阻塞，不得宣称算法失败或精度达标 |

P0 的第一阶段只验证 `camera → target_link` 相对结果；第二阶段才接入 `base_link → camera → map`。这样可以把视觉算法误差和平台 UWB 误差分开。

### P1：平台坐标链与可选融合

在 P0 可重复后，接入官方 UWB/飞控平台位姿，完成时间配对、外参变换和 `map → target_link`。只有原始 UWB 距离、时间同步和独立真值均可用时，才考虑 UVINS/GTSAM 或其他紧耦合融合。

### P2：AprilTag 延后基线

使用现有 `apriltag_ros` + AprilTag 36h11 + IPPE/PnP，复用 `TargetObservation` 输出。标签边长、`tag_target → target_link` 外参、相机内参和时间戳必须实测；检测成功不等于世界坐标精度达标。

### 硬件与标定准入

先验收机载相机是否能提供原始 `Image`、`CameraInfo`、采样时间和光学 frame；再记录 FanciSwarm 实际 UWB/飞控字段、单位、时间基准、更新率和标签安装对象。`map` 使用右手系、`+z` 向上，原点与 `+x/+y` 基准由沙盘实测确定。相机内参、`base_link → camera_i`、UWB 基站坐标和目标几何外参都必须关联标定运行编号，并用独立验证点复核；缺少这些证据时只做相对视觉或接口回放。

## 3. 当前数据接口（代码基准）

### ROS 话题

| 层 | 话题 | 消息 | 语义/状态 |
|---|---|---|---|
| ROS1 飞控 | `/odom_global_001` | `nav_msgs/Odometry` | 无人机平台位姿；不能当机外目标 |
| ROS1 飞控 | `/imu_global_001` | `sensor_msgs/Imu` | 平台 IMU；时间/单位以适配器验收为准 |
| bridge | `/nexus/fcu/odom` | `nav_msgs/msg/Odometry` | ROS1 平台位姿映射 |
| bridge | `/nexus/fcu/imu` | `sensor_msgs/msg/Imu` | ROS1 IMU 映射 |
| ROS2 视觉 | `/nexus/vision/target_observation` | `nexus_msgs/msg/TargetObservation` | `camera_i` 下的机外目标观测；当前实现为 AprilTag 适配器，P0 无标签适配器待实现 |
| 坐标变换 | `/nexus/vision/map_target_observation` | `nexus_msgs/msg/TargetObservation` | 已验证 TF 变换到 `map` 的目标观测 |
| UWB 目标输入 | `/nexus/uwb/raw_target_observation` | `nexus_msgs/msg/TargetObservation` | 仅在明确 UWB 直接测量机外目标时启用；当前 UWB 默认测平台 |
| 坐标变换 | `/nexus/uwb/target_observation` | `nexus_msgs/msg/TargetObservation` | UWB 目标观测的 `map` 结果；未验收时不发布猜测值 |
| 主输出 | `/nexus/target/pose` | `nexus_msgs/msg/TargetObservation` | 机外目标 `map → target_link`；融合或单源降级均需保留来源模式 |

### `TargetObservation.msg`

字段以 `code/ros2_ws/src/nexus_msgs/msg/TargetObservation.msg` 为准：`header.stamp` 是采样时间；`receive_timestamp_ns` 是当前层接收/形成时间；`target_id`、`source_mode`、`validity`、`invalid_reason`、`last_valid_sample_timestamp_ns`、`unit="m"`、`pose`、36 元素协方差和 0–1 `confidence` 必须存在。无效消息使用 NaN 占位，不能复用上一帧。

来源常量：`SOURCE_DIRECT_UWB=1`、`SOURCE_DIRECT_VISION=2`、`SOURCE_PLATFORM_RELATIVE=3`、`SOURCE_FUSED=4`。有效性常量：`UNKNOWN=0`、`VALID=1`、`INVALID=2`。

### transport envelope

`code/tools/nexus_channel_contract.py` 是唯一校验实现。必须包含 `schema_version=1`、`message_type`、连续 `sequence`、正的 `sample_timestamp_ns`、不早于采样时间的 `receive_timestamp_ns`、非空 `frame_id`、来源模式、有效性、原因和 `payload`。目标 payload 必须有 `target_id`、米制 `unit`、3×3/9 元素协方差和 0–1 `confidence`；时效由调用方传入的 `now_timestamp_ns`/`max_age_ns` 检查，不把实验阈值写死在 envelope。

## 4. 最小验收矩阵

每次正式运行都记录：算法和代码提交、权重/模型、相机型号与分辨率、`CameraInfo`、目标几何/地图版本、`base_link → camera` 外参、UWB 基站坐标与单位、时间基准、真值来源、命令和输出 SHA256。

至少报告：目标平移 RMSE/P95、旋转误差（如适用）、重投影误差、可用率、端到端延迟、失锁恢复时间和失败原因；分别列出纯视觉、平台 UWB 和坐标链误差。

## 5. 不得做的事

- 不把规划目标、论文数字、厂商标称或 AprilTag 结果写成无标签算法实测精度。
- 不把 `/odom_global_001`、UWB 标签平台位置或上一帧目标坐标改名为机外目标。
- 不在缺少图像采样时间、内参、TF、单位或目标几何时发布猜测坐标。
- 不为追求文档完整性新增重复计划；新事实只更新本基线、代码或 `experiments/runs/`。
