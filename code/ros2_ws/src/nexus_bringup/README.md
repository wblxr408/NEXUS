# nexus_bringup（ROS2）

职责：集中管理 ROS2 参数、`ros2 launch`、`ros2 bag` 录制/回放和演示启动。主启动顺序是跨 ROS/离线输入 → 目标视觉观测 → `tf2`/目标坐标求解与融合 → `/nexus/target/pose` → RViz2/评估与视频录制；ROS1 侧 `fcu_core` 单独按 `ros1_ws` 启动。

`target_localization.launch.py` 默认加载空的到货前外参模板并启动 RViz2。硬件到货前没有 `map` 实测外参，因此视觉世界观测保持无效是预期行为；不得在模板中填写猜测的沙盘原点或轴向。无图形环境可用 `use_rviz:=false`，不影响坐标与融合节点。
