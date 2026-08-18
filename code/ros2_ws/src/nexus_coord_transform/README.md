# nexus_coord_transform（ROS2）

职责：维护 `map`、相机、UWB 和无人机机体坐标系，发布 ROS2 `tf2` 变换。ROS1 侧仍使用 `tf`，跨 bridge 时按接口表转换。所有外参必须关联标定运行编号，并显式记录 NED/ENU 与轴向转换。
