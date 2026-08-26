# Gazebo 无人机连续航线记录

日期：2026-08-26  
范围：`simulation/nexus_sandbox_imx219.world`、`code/ros2_ws/src/nexus_bringup/`

## 实现

Gazebo 中的 `nexus_uav` 从固定模型改为可运动模型，使用
`libgazebo_ros_planar_move.so` 接收 `/nexus/gazebo/uav/cmd_vel`，并从同一插件发布
`/nexus/gazebo/uav/odom`。`uav_route_node` 默认以约 `0.55 m/s` 在以下 `map` 航点循环：

`(0.70, 0.70)` → `(3.30, 0.70)` → `(3.30, 4.00)` → `(0.70, 4.00)`。

模型高度保持 `z=2.50 m`；此功能用于连续的视觉、UWB 和前端演示输入，不模拟真实飞控、三维动力学或避障。

## 启动与手动控制

```bash
ros2 launch nexus_bringup gazebo_sandbox.launch.py use_gui:=true follow_route:=true
```

设置 `follow_route:=false` 后，可向 `/nexus/gazebo/uav/cmd_vel` 发布 `geometry_msgs/Twist` 手动驱动。

## 验证

- `python3 -m py_compile src/nexus_bringup/src/nexus_bringup/uav_route_node.py` 通过。
- `colcon build --packages-select nexus_bringup` 通过。
- `colcon test --packages-select nexus_bringup` 通过（14 passed）。
- 无 GUI 实测：启动 Gazebo 与 `follow_route:=true`，先后采集 `/nexus/gazebo/uav/odom`。5 秒内 XY 位移为 `0.678 m`，两次高度均为 `2.50 m`。
