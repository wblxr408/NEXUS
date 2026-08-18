# nexus_bringup（ROS2）

职责：集中管理 ROS2 参数、`ros2 launch`、`ros2 bag` 录制/回放和演示启动。主启动顺序是 bridge 输入 → 视觉 → `tf2`/融合 → RViz2/评估；ROS1 侧 `fcu_core` 单独按 `ros1_ws` 启动。
