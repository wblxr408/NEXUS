# nexus_fusion_localization（ROS2）

职责：以 ROS1 bridge 转入的幻思飞控 UWB/光流定位为飞行安全基线；ROS2 端先实现视觉位姿回传和轨迹评估，再按实验需要增加轻量 EKF/自适应修正。不要一开始替换飞控的完整位置闭环。
