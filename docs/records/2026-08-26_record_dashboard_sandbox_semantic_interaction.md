# Dashboard 沙盘语义悬浮交互记录

日期：2026-08-26  
范围：`code/ros2_ws/src/nexus_viz_dashboard/web/`

## 变更

- 为 Three.js 沙盘中的参数化物体附加可拾取的语义信息；鼠标悬停时，地图右上角固定信息窗显示名称、类型、`map` 世界坐标与说明。
- 坐标统一以米显示到小数点后两位（厘米）；空闲状态显示沙盘范围：`x=0.00–4.00 m`、`y=0.00–4.70 m`、`+z` 向上。
- 增加四角 UWB 基站示意：`(0.15, 0.15, 0.70)`、`(3.85, 0.15, 0.70)`、`(3.85, 4.55, 0.70)`、`(0.15, 4.55, 0.70)`，单位为 `m`。
- 增加模拟无人机、Gazebo/IMX219 相机与机载 UWB 标签；无人机存在 `/nexus/gazebo/uav/odom` 位置时跟随该位置，否则使用静态示意位置 `(2.00, 2.35, 1.20) m`。

## 坐标与证据边界

`map` 的物理原点、设备外参与 UWB 基站最终坐标仍须由现场标定记录确认。四角位置、0.70 m 高度和静态无人机位置仅为前端展示的标称布局，不能作为厘米级定位精度或实测安装结果。

## 验证

- 2026-08-26：`git diff --check` 通过。
- 2026-08-26：`cd code/ros2_ws && colcon build --packages-select nexus_viz_dashboard` 通过。
- 2026-08-26：`cd code/ros2_ws && colcon test --packages-select nexus_viz_dashboard` 通过（`test_dashboard_contract`：1 passed）。
- 2026-08-26：尝试以 `node --check` 作浏览器模块语法检查；环境未安装 `node`（`node: command not found`），未执行该项。
- 未启动浏览器、Gazebo 或 rosbridge；未验证实际鼠标拾取和实时无人机位姿更新。
