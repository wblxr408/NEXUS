# ROS1/ROS2 接口表（幻思 fcu_core 适配草案）

| 层 | Topic/服务 | 消息类型 | 频率 | frame | 状态 |
|---|---|---|---:|---|---|
| ROS1 飞控/UWB | `/odom_global_001` | `nav_msgs/Odometry` | 厂商配置 | `map`/`uwb` | `fcu_core` 官方话题 |
| ROS1 飞控状态 | `/imu_global_001` | `sensor_msgs/Imu` | 厂商配置 | `scaled_imu` | `fcu_core` 官方话题 |
| ROS1→ROS2 bridge | `/nexus/fcu/odom` | `nav_msgs/msg/Odometry` | bridge 配置 | `map`/`uwb` | 项目草案，待验证 |
| ROS1→ROS2 bridge | `/nexus/fcu/imu` | `sensor_msgs/msg/Imu` | bridge 配置 | `scaled_imu` | 项目草案，待验证 |
| UWB原始距离 | `TBD` | `TBD` | TBD | `uwb` | 先确认厂商是否开放；不开放则不自写解算器 |
| ROS2 视觉 | `/nexus/vision/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | TBD | `map` | ROS2 项目话题，待相机验证 |
| ROS2→ROS1 bridge | `/motion_001` | `geometry_msgs/PoseStamped` | TBD | `map` | ROS1 `fcu_core` 接收并转发给飞控 |
| ROS2 评估 | `/nexus/evaluation/reference` | `TBD` | TBD | `map` | 待真值方案确认 |

官方接口优先复用 `fcu_core`；任何接口变更新增 `docs/decisions/YYYY-MM-DD_ADR-xxx_interface-*.md`，并同步节点 README。ROS1 使用 `tf`/`rosbag`，ROS2 使用 `tf2`/`ros2 bag`；消息映射必须单独记录。
