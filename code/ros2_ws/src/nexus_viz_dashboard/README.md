# nexus_viz_dashboard（ROS2）

职责：RViz2、ROS2 rosbridge 接入和 Web Dashboard 展示，明确区分无人机轨迹、目标真值、UWB/视觉目标观测和 `/nexus/target/pose`。页面展示的指标必须带运行编号，不得把规划目标渲染成实测值。ROS1 录包由硬件兼容层负责，ROS2 侧统一使用 `ros2 bag` 做算法回放和演示视频素材。

浏览器端目录骨架见 [`web/`](web/README.md)。当前不含前端实现或模拟数据；消息与部署方案须在硬件接口确认后固化。
