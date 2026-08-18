# nexus_vision_localization（ROS2）

职责：接入外部多相机或 FanciSwarm 视觉套件的原始图像，完成标签检测、标定和三角测量，输出 ROS2 `nexus/vision/pose`，再由 bridge 回传 ROS1 `motion_001`。相机数量、标签族和标定版本由配置指定；飞控板上的 800 万像素相机只有在确认原始视频接口后才能使用。
