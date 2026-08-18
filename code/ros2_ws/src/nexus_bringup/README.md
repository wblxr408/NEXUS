# nexus_bringup（ROS2）

职责：集中管理 ROS2 参数、`ros2 launch`、`ros2 bag` 录制/回放和演示启动。主启动顺序是跨 ROS/离线输入 → 目标视觉观测 → `tf2`/目标坐标求解与融合 → `/nexus/target/pose` → RViz2/评估与视频录制；ROS1 侧 `fcu_core` 单独按 `ros1_ws` 启动。
