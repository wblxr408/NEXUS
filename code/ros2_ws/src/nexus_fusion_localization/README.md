# nexus_fusion_localization（ROS2）

职责：比较和组合可用的 UWB 目标观测、无人机平台位姿与视觉目标观测，统一输出 `/nexus/target/pose` 并评估目标定位误差。先实现可解释的单源基线和坐标求解，再按实验需要增加轻量 EKF/自适应修正；不替换飞控自身的位置闭环。
