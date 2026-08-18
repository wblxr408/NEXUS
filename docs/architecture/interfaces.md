# ROS1/ROS2 接口表（幻思 fcu_core 适配草案）

范围：最终输出是无人机之外目标的世界坐标。`/odom_global_001` 默认只作为无人机/飞行平台状态理解，除非硬件验收明确证明其 UWB 标签和消息语义对应目标。决定见 `ADR-002`。

| 层 | Topic/服务 | 消息类型 | 频率 | frame | 状态 |
|---|---|---|---:|---|---|
| ROS1 飞控/UWB | `/odom_global_001` | `nav_msgs/Odometry` | 厂商配置 | `map`/`uwb` | `fcu_core` 官方话题；平台位姿语义待实机确认 |
| ROS1 飞控状态 | `/imu_global_001` | `sensor_msgs/Imu` | 厂商配置 | `scaled_imu` | `fcu_core` 官方话题 |
| ROS1→ROS2 bridge | `/nexus/fcu/odom` | `nav_msgs/msg/Odometry` | bridge 配置 | `map`/`uwb` | 项目草案，待验证 |
| ROS1→ROS2 bridge | `/nexus/fcu/imu` | `sensor_msgs/msg/Imu` | bridge 配置 | `scaled_imu` | 项目草案，待验证 |
| UWB目标观测/原始距离 | `TBD` | `TBD` | TBD | `uwb` | 确认标签安装对象及厂商开放范围后定义 |
| ROS2 视觉目标观测 | `/nexus/vision/target_observation` | `nexus_msgs/msg/TargetObservation` | TBD | `camera_i`/`map` | 待相机和测量模型验证 |
| ROS2 目标定位输出 | `/nexus/target/pose` | `nexus_msgs/msg/TargetObservation` | TBD | `map` | 赛题主输出；消息契约已定义，真实目标字段待硬件确认 |
| ROS2→ROS1 可选回传 | `/motion_001` | `geometry_msgs/PoseStamped` | TBD | `map` | 仅目标跟踪控制需要时启用；不是首版验收条件 |
| ROS2 评估 | `/nexus/evaluation/reference` | `TBD` | TBD | `map` | 待真值方案确认 |

官方接口优先复用 `fcu_core`；任何接口变更新增 `docs/decisions/YYYY-MM-DD_ADR-xxx_interface-*.md`，并同步节点 README。ROS1 使用 `tf`/`rosbag`，ROS2 使用 `tf2`/`ros2 bag`；消息映射必须单独记录。传输无关 envelope 由 `code/tools/nexus_channel_contract.py` 校验。禁止把 `/nexus/fcu/odom` 直接重命名成目标位姿。

envelope 的 `sequence` 从 0 开始并按消息流递增 1；`sample_timestamp_ns` 是采样时刻而非转发时刻，必须为正整数；`frame_id` 缺失即拒绝。目标观测 payload 还必须包含 `target_id`、`unit: m`、3×3 位置协方差和 0–1 `confidence`。允许的来源模式为 1–4（或对应 `SOURCE_*` 名称）；不支持的 schema 版本和异常字段由适配层明确拒绝。平台状态 envelope 不得凭字段名称自动变成目标观测。
