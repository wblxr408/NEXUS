# ROS2 第三方模块适配说明

本说明只定义将来适配边界，不修改 `apriltag_ros/` 或 `rosbridge_suite/` 的上游源码。

## `apriltag_ros`

当前状态：直接下载，尚未加入 `colcon` 构建或启动。

启用前确认：真实相机可提供校正后的 `sensor_msgs/Image`、匹配的 `sensor_msgs/CameraInfo`、相机 `frame_id`、标签家族和实体标签边长（米）。

将来只在项目自有 `nexus_vision_localization` 中做适配：通过 launch 参数重映射图像和相机内参话题；将上游检测结果转换为 `nexus_msgs/msg/TargetObservation`；补充采样时间、`target_id`、协方差、置信度以及到 `map` 的 `tf2` 变换。不得改动上游检测、PnP 或消息定义，不得把检测到的标签默认解释成无人机或赛题目标。

## `rosbridge_suite`

当前状态：直接下载，尚未加入 `colcon` 构建或对浏览器开放端口。

启用前确认：ROS2 Humble 与上游当前依赖兼容；Dashboard 的访问网络和端口策略已确定；只暴露获准的话题。

将来只在 `nexus_viz_dashboard` 的启动和网页配置中适配：订阅并展示 `/nexus/target/pose`、来源、时间延迟和带运行编号的评估结果；设置允许的主题白名单。不得在 rosbridge 或浏览器端实现坐标变换、融合、指标计算，也不得展示规划精度为实测值。
