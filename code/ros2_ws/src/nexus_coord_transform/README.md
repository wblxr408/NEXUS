# nexus_coord_transform（ROS2）

职责：维护 `map`、相机、UWB、无人机 `base_link` 和外部目标 `target_link` 坐标系，发布 ROS2 `tf2` 变换并把目标观测转换为目标世界坐标。ROS1 侧仍使用 `tf`，跨信息通道时按接口表转换。所有外参必须关联标定运行编号，并显式记录 NED/ENU、轴向以及 UWB 标签安装对象。

实时入口为 `/nexus/vision/target_observation` 和 `/nexus/uwb/raw_target_observation`，输出为各自的 `map` 观测。节点按观测采样时间查询 tf2，并旋转位置协方差；缺少外参时发布 `transform_unavailable` 无效状态。`nexus_bringup/config/pre_hardware_transforms.yaml` 故意不含沙盘坐标，硬件到货后才在标定运行中填写实测外参。
