# E044 目标选取到坐标输出全链路审核

日期：2026-09-08。结论：**PARTIALLY COMPLETE，不能保证当前实机上电即贯通**。用户确认停机原因是无人机电池耗尽，本轮不再尝试访问未供电设备。审核范围是当前代码、启动连接和可执行软件验证，不修改生产代码或飞控设置。

## 主链路

网页定位图像框选 → `/nexus/vision/target_reference_image` 与 `/nexus/vision/target_requests` → SuperPoint 参考注册/身份关联与检测辅助 → `/nexus/vision/target_tracks` → target_metric_node（CameraInfo、机体到相机外参、同步平台位姿、多帧几何）→ `/nexus/vision/target_kinematics` → target_kinematic_fusion_node → `/nexus/target/kinematics` → localization_dashboard_node → `/nexus/viz/localization_state` 与 RViz markers → 网页/RViz。

相关消息名与发布/订阅在当前源码中对应；有软件级 ROS 测试。框选注册并不直接产生米制坐标，身份匹配成功也不等于深度可观测。不能用演示坐标或静态目标地图坐标替代实际几何输出。

## 发现与阻断项

1. **当前平台近似配置不能经统一入口加载。** `nexus_bringup/launch/dual_localization.launch.py` 调用 `load_platform_calibration(path, allow_test)`，没有传递近似许可；E042 配置状态是 experimental_approximation。以同一加载方式实际执行报 ValueError，退出 1，证据为 current_calibration_launch_gate.txt。这是当前配置与统一启动接线不兼容，不是构建失败；保持校准门限是有意保护，不应通过改成 measured 绕过。

2. **现场 IMU 话题与平台订阅不同，当前部署未闭环。** 断电前 Pi 图中飞控发布 `/imu_global_001`，平台节点固定订阅 `/nexus/fcu/imu`。独立 JSON ingress 或重映射可实现转接，但断电前现场没有该 ingress 进程，统一主入口默认 telemetry_udp=false，且未包含 Pi 图像/遥测入口。应明确一套实际使用的入口、域 ID 和启动命令；改话题名还需核对 frame、时间和单位，不能凭重命名视为全部标定完成。

3. **平台缺少已验证的初始化来源。** platform_node._drain 在 estimator.states 为空时直接返回；初始化来自 `/nexus/platform/initial_pose` 或固定参考完整位姿。原始 IMU/UWB 距离本身不自动初始化。统一入口强制需要 reference_calibration 并启动 AprilTag/fixed_reference；尚无本轮完整实机初始位姿注入及后续持续平台 odom 验收记录。目标几何依赖 `/nexus/platform/odom`，缺失时不能计算有效 map 坐标。E042 当前近似平台融合真实回放也未通过连续性验收。

4. **200 组平滑不是目标定位后端已接入的观测。** E043 新增 `/nexus/uwb/ranges_smoothed` 为独立静置统计输出；platform_node 仍读 `/nexus/uwb/ranges`。这符合 E043 保留原始链路的实现，但不能推导为定位结果已经采用平滑。长窗口统计具有约 200 s 历史跨度、不同槽保留样本不同，且不含瞬时观测 variances_m2；不能只改 remapping 把它当作当前位姿测量。

5. **真实目标识别与米制几何仍缺联合验收。** E038 已证明真实 IMX219 图像、CameraInfo 可进入 ROS，不应说相机从未接通；但当时机架遮挡、目标未完整入镜。E037 的实体参考图仍 pending、训练证据主要为 RRD 仿真域。网页可直接采集参考图，不必预先拍齐 517 个实例，但至少需要一个实际目标完成注册、跟踪、几何有效与最终坐标显示的连续记录。已知相机内参不能替代机体到相机外参和平台初始位姿；没有足够运动基线时多帧几何也可能拒绝解。

## 可确认的完成项

|环节|已确认|实际边界|
|---|---|---|
|选目标|网页定格、原图像素框选、参考图/请求发送、注册回执和失败处理|本轮无实体目标注册回执|
|识别/跟踪|检测、参考特征关联、身份与目标轨迹输出有实现和测试|不保证任意实体目标在遮挡或弱纹理条件下被识别|
|计算坐标|平台+相机+多帧目标几何与融合有实现和 ROS 测试|实机平台初始化/启动输入和几何条件未闭环|
|输出|目标融合消息、网页快照和 RViz 输出对应|无本轮真实目标坐标端到端验收|

## 验证

在 WSL Ubuntu22.04 / ROS2 Humble 执行：

```bash
source /opt/ros/humble/setup.bash
cd code/ros2_ws
colcon build --packages-select nexus_bringup nexus_vision_localization nexus_fusion_localization nexus_viz_dashboard
FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA colcon test --packages-select nexus_bringup nexus_vision_localization nexus_fusion_localization nexus_viz_dashboard
```

四包构建通过。逐包 xunit 统计：bringup 17、vision 86、fusion 81、dashboard 8，共 **192 测试、0 失败、0 错误、0 跳过**。selected_package_test_counts.json 保存统计。ros_validation.txt 末尾全 build 范围的 268/6 skipped 包括其他包旧结果，不能当作本轮新跑数量。

仓库根目录：

```bash
node --experimental-default-type=module --test code/ros2_ws/src/nexus_viz_dashboard/web/test/registration.test.mjs code/ros2_ws/src/nexus_viz_dashboard/web/test/localization.test.mjs
```

网页 11 项通过，见 web_validation.txt。本轮未启动浏览器实景测试。配置加载复现明确失败，未隐藏、未放宽门限。

## 后续验收顺序

先明确单一实机启动链，接通 IMU/相机并解决近似配置入口和完整平台初始位姿；再用一个清晰入镜的目标完成“框选注册成功 → 身份持续匹配 → 目标几何有效 → map 米制坐标 → 网页/RViz 显示”的同一次录包。静置平滑可显示和记录，但不能替代运动平台定位。用户要求审核，本轮仅新增审核证据，未擅自进行上述修复。

未扩大任务范围。
