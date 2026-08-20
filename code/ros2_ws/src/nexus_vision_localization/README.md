# nexus_vision_localization（ROS2）

职责：接入 `apriltag_ros` 的 AprilTag 36h11 检测结果和同一相机的 `CameraInfo`，对首版 `tag_id: 0` 做 PnP，输出外部目标 `target_0` 的 `/nexus/vision/target_observation`。输出必须注明相机/世界 frame、采样时间、目标标识和不确定度，不能与无人机自身位姿混用。

`marker_size_m` 是到货后用量具复测的实体 AprilTag 边长，默认 `0.0` 表示未配置，节点会拒绝输出；不得用上游示例尺寸代替。`target_link` 是刚性安装板外向平面的几何中心，标签中心到该点的外参必须在标定运行中登记。相机输入、`CameraInfo`、frame 和时间戳确认后，使用 `apriltag_localization.launch.py` 同时启动 `apriltag_ros` 与本适配节点；适配节点拒绝相机参数与检测时间戳或 frame 不一致的数据。完整决定见 `ADR-004`。
