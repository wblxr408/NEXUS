# ROS1/ROS2 接口表（当前代码基准）

状态：current（2026-08-25）。字段和校验以 `code/ros2_ws/src/nexus_msgs/msg/TargetObservation.msg`、`code/tools/nexus_channel_contract.py` 及各包 `config/*.yaml` 为准；本表只解释语义。算法复现顺序见 [当前执行基线](../2026-08-25_plan_current_execution_baseline.md)。

## 话题

| 层 | Topic | 消息类型 | frame/语义 | 状态 |
|---|---|---|---|---|
| ROS1 飞控 | `/odom_global_001` | `nav_msgs/Odometry` | 无人机平台位姿（`base_link` 语义）；不是机外目标 | 官方接口，字段映射待实机验收 |
| ROS1 飞控 | `/imu_global_001` | `sensor_msgs/Imu` | 平台 IMU | 官方接口，时间/单位待实机验收 |
| ROS1→ROS2 bridge | `/nexus/fcu/odom` | `nav_msgs/msg/Odometry` | 平台状态 | 代码映射；不改名为目标 |
| ROS1→ROS2 bridge | `/nexus/fcu/imu` | `sensor_msgs/msg/Imu` | 平台 IMU | 代码映射；不替代目标观测 |
| ROS2 视觉 | `/nexus/vision/target_observation` | `nexus_msgs/msg/TargetObservation` | `camera_i` 下的机外目标 | 当前节点为 AprilTag/PnP；无标签适配器必须复用此契约 |
| ROS2 坐标变换 | `/nexus/vision/map_target_observation` | `nexus_msgs/msg/TargetObservation` | `map` 下的机外目标 | 采样时刻 TF 未验证则发布无效状态，不猜坐标 |
| ROS2 UWB 输入 | `/nexus/uwb/raw_target_observation` | `nexus_msgs/msg/TargetObservation` | 原始 UWB 目标观测 | 仅直接测量机外目标时启用；当前 UWB 默认测无人机平台 |
| ROS2 坐标变换 | `/nexus/uwb/target_observation` | `nexus_msgs/msg/TargetObservation` | `map` 下的 UWB 目标 | 同上，不能由平台 UWB 自动生成 |
| ROS2 主输出 | `/nexus/target/pose` | `nexus_msgs/msg/TargetObservation` | `map → target_link` | 融合或单源降级；保留 `source_mode` |
| ROS2→ROS1 可选 | `/motion_001` | `geometry_msgs/PoseStamped` | 控制回传 | 非首版验收条件 |

## `TargetObservation` 契约

消息定义中的常量固定为：`SOURCE_DIRECT_UWB=1`、`SOURCE_DIRECT_VISION=2`、`SOURCE_PLATFORM_RELATIVE=3`、`SOURCE_FUSED=4`；`VALIDITY_UNKNOWN=0`、`VALIDITY_VALID=1`、`VALIDITY_INVALID=2`。

必填语义：

- `header.stamp` 是采样时间，不得用接收时间替代；`header.frame_id` 必须非空。
- `receive_timestamp_ns` 是当前层接收/形成时间，不能早于采样时间。
- `target_id` 永远是机外目标标识；平台位姿不得填入此消息。
- `unit` 固定为米（`m`）；`pose` 和 36 元素协方差使用 `float64`。
- 有效消息的 `invalid_reason` 必须为空；无效消息必须给出原因，并用 NaN 占位，不能复用上一帧。
- `last_valid_sample_timestamp_ns` 只用于诊断；`confidence` 必须在 0～1。

## transport envelope

`nexus_channel_contract.py` 要求 `schema_version=1`、`message_type`、从 0 连续递增的 `sequence`、正的 `sample_timestamp_ns`、不早于采样时间的 `receive_timestamp_ns`、非空 `frame_id`、来源模式、有效性、原因和 `payload`。目标 payload 还必须有 `target_id`、米制 `unit`、3×3/9 元素协方差和 0～1 `confidence`。`validate_freshness` 的 `now_timestamp_ns` 与 `max_age_ns` 由调用方冻结，不由协议偷偷设定实验阈值。

## 坐标和边界

ROS1 使用 `tf`，ROS2 使用 `tf2`；`map`、`base_link`、`camera_i` 和 `target_link` 的物理外参只能来自标定运行。UWB 输出如只代表无人机标签，必须先进入平台状态接口，再通过 `base_link → camera_i` 与视觉目标观测得到 `map → target_link`。禁止把 `/odom_global_001`、`/nexus/fcu/odom` 或 UWB 标签平台位置直接重命名为目标位姿。
