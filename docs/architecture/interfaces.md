# ROS2 接口表（草案）

| 层 | Topic/服务 | 消息类型 | 频率 | frame | 状态 |
|---|---|---|---:|---|---|
| UWB | `/nexus/uwb/ranges` | `TBD` | TBD | `uwb_anchor_*` | 待硬件确认 |
| 视觉 | `/nexus/vision/pose` | `geometry_msgs/PoseWithCovarianceStamped` | TBD | `world` | 待相机验证 |
| 融合 | `/nexus/localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` | TBD | `world` | 基线接口 |
| 评估 | `/nexus/evaluation/reference` | `TBD` | TBD | `world` | 待真值方案确认 |

任何接口变更新增 `docs/decisions/YYYY-MM-DD_ADR-xxx_interface-*.md`，并同步节点 README。
