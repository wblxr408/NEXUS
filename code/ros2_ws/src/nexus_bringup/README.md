# nexus_bringup（ROS2）

职责：集中管理 ROS2 参数、`ros2 launch`、`ros2 bag` 录制/回放和演示启动。主启动顺序是跨 ROS/离线输入 → 目标视觉观测 → `tf2`/目标坐标求解与融合 → `/nexus/target/pose` → RViz2/评估与视频录制；ROS1 侧 `fcu_core` 单独按 `ros1_ws` 启动。

`target_localization.launch.py` 默认加载空的到货前外参模板并启动 RViz2。硬件到货前没有 `map` 实测外参，因此视觉世界观测保持无效是预期行为；不得在模板中填写猜测的沙盘原点或轴向。无图形环境可用 `use_rviz:=false`，不影响坐标与融合节点。

## Gazebo 沙盘与前端

`gazebo_sandbox.launch.py` 加载从 `simulation/sandbox_scene.yaml` 生成的
`nexus_sandbox_imx219.world`。它和前端使用同一份沙盘生成器输出，世界坐标为
`map` 的米制 `x/y/z`；世界中包含一个保持 `z=2.5 m` 的模拟无人机观察点，便于先验证图像、坐标和浏览器链路，不代表飞控动力学或实测精度。

先在仓库根目录更新派生文件并构建：

```bash
python3 simulation/generate_sandbox.py --scene simulation/sandbox_scene.yaml --out simulation
colcon build --packages-select nexus_bringup
source install/setup.bash
ros2 launch nexus_bringup gazebo_sandbox.launch.py use_gui:=true follow_route:=true
```

`follow_route:=true` 时，`uav_route_node` 会以约 `0.55 m/s` 连续循环经过
`(0.70, 0.70)`、`(3.30, 0.70)`、`(3.30, 4.00)`、`(0.70, 4.00)`（`map`、m）；它通过
`/nexus/gazebo/uav/cmd_vel` 驱动可运动的 Gazebo 模型。需要手动控制时以
`follow_route:=false` 启动并向同一话题发布 `geometry_msgs/Twist`。

IMX219 安装在机身下方，镜头固定朝 `-z`，因此输出的是下方沙盘物体。Gazebo 相机发布 `/nexus/camera/imx219/image_raw` 和
`/nexus/camera/imx219/camera_info`；无人机模拟真值位置发布
`/nexus/gazebo/uav/odom`。前端通过 rosbridge 订阅这些话题及现有定位话题；在另一个
已安装/构建 `rosbridge_server` 的终端运行 `ros2 run rosbridge_server rosbridge_websocket`，
再按 `nexus_viz_dashboard/web/README.md` 启动静态页面。若 rosbridge 不在默认端口，可用
`?rosbridge=ws://<host>:<port>` 指定。浏览器为避免 8 MP 原始图像占满链路，按 1 Hz 拉取并
下采样预览；算法节点仍订阅原始 ROS 图像。
