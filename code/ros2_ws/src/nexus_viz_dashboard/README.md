# nexus_viz_dashboard（ROS2）

职责：RViz2、ROS2 rosbridge 接入和 Web Dashboard 展示，明确区分无人机轨迹、目标真值、UWB/视觉目标观测和 `/nexus/target/pose`。页面展示的指标必须带运行编号，不得把规划目标渲染成实测值。ROS1 录包由硬件兼容层负责，ROS2 侧统一使用 `ros2 bag` 做算法回放和演示视频素材。
