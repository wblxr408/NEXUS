# 六项静态目标修正：实现与验证记录

> 后续状态更新：用户指定的录包、旧回归和 Web/RViz 整链补验已完成，最新结果见[本机闭环验证记录](2026-09-03_record_local_integration_validation.md)。下文保留初次修正时的失败与未验证记录，不能用来覆盖后续证据。实机精度仍未验证。

日期：2026-09-03。依据：[ADR-013](decisions/2026-09-03_ADR-013_static_targets_readonly_localization.md)。

当前整体状态：**PARTIALLY COMPLETE**。第 1–4 项已有软件验证；第 5 项统一入口已实现，但实际 rosbag 回环未通过；第 6 项已更新测试和记录，回归中的失败保持可见。以下均不是实机精度结论。

## 实现清单

| 项目 | 代码位置（相对 `code/ros2_ws/src/`） | 当前行为与证据 | 状态 |
|---|---|---|---|
| 1. 静态目标 | `nexus_vision_localization` 的 `target_identity.py`、`target_frontend.py`、`superpoint_node.py`、`target_metric_node.py`、`target_multiview.py` | 注册默认 static；在线几何拒绝匀速目标模型；所有有效视图共同估计一个 map 三维点；保留无人机运动引起的像素跟踪 | VALIDATED（合成软件） |
| 2. 融合与遮挡 | `nexus_fusion_localization` 的 `kinematic_filter.py`、`target_kinematic_node.py`；`nexus_msgs/msg/TargetKinematicState.msg` | 不做目标速度外推、不添加目标加速度过程噪声；遮挡保留原位置/协方差与原采样时间，显式 historical；地图历史受数量限制；重捕获必须通过静态锚点与身份检查 | VALIDATED（数值与 DDS） |
| 3. 条件误差 | `target_multiview.py`、`target_geometry_template.yaml` | 使用实际平台位姿/安装变换，但目标协方差只计视觉噪声及鲁棒权重；平台估计协方差本身保留；CI 保留重叠视图相关性处理 | VALIDATED（独立有限差分与噪声缩放对照） |
| 4. 双对象展示 | `nexus_viz_dashboard` 的 `localization_dashboard_node.py`、`rviz/nexus_dual_localization.rviz`、Web `dual_localization.js` / `localization_state.js` 及网关/store | UAV 实际 p/q/速度与静态目标分别显示；当前、历史、丢失、重新识别明确标记；无输入/过期清空当前值；历史不冒充测量；Web 只订阅 | VALIDATED（DDS 消息、Web 测试与 Edge 页面检查）；RViz 实际渲染未验 |
| 5. 总启动与记录 | `nexus_bringup/launch/dual_localization.launch.py`、`dual_recording.py` | 新定位主入口组装平台/视觉/目标/展示；无飞行控制节点；显式录包、禁止覆盖，保存输入配置/哈希/代码版本；录包 topics 含传感器、双对象结果及质量状态 | IMPLEMENTED / UNVERIFIED：录包回环失败 |
| 6. 测试与口径 | 三个算法/显示包的相关测试、bringup 新测试、本记录及 ADR-013 | 测试静态目标、无人机变速变高、遮挡/长时恢复、同参考点错误重关联、异常观测、历史显示和只读启动；CV 目标成功用例改为拒绝用例 | 测试与记录已更新；回归未全通过 |

无人机的速度和高度由已有操控方式决定。新定位链路只接收观测和估计状态，不发布速度、高度、航点、解锁或模式指令。

## 软件验证

环境：Ubuntu 22.04 / WSL、Python 3.10、ROS2 Humble、现有 NumPy/SciPy/OpenCV、Node 22、Windows Edge。未安装依赖或模型、未接管真实设备。

```bash
cd code/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to nexus_bringup --symlink-install
source install/setup.bash
OPENBLAS_NUM_THREADS=1 colcon test --packages-select nexus_msgs nexus_fusion_localization nexus_vision_localization nexus_viz_dashboard nexus_bringup
colcon test-result --verbose
```

- 最终整体构建：7 个包成功，包括工作区 AprilTag 包与坐标转换依赖。
- 五个包的 xunit：msgs 3、fusion 79、vision 67、viz 8、bringup 16，共 **173 个 Python/launch 用例，171 通过、2 失败、0 error**。Node 单独包含 3 个 Web 行为测试，全部通过；不把 CTest 包装项重复算作独立用例。
- 算法覆盖包含实际姿态/变速/变高输入，但图像网络输出与平台测试输入为合成夹具，不是实际网络和飞行数据。
- Edge 使用明确 `SIMULATION / synthetic_browser_qa` 数据检查无输入、当前观测、历史显示；桌面 1440×1000 与手机宽度 390 无脚本错误、无横向溢出；人工检查截图后修正纵向遮挡与地图文字裁剪。浏览器检查通过 NexusAPI 注入状态，不能替代 ROS2→rosbridge→浏览器的整链验收。
- Web 桥接启动检查曾发现 glob 参数类型错误，已改为显式字符串；默认 9090 端口占用时没有终止其他服务。使用 `rosbridge_port:=19091` 的真实 launch action 启动成功并正常退出。
- 末次改动后定向执行 `python3 -m pytest src/nexus_bringup/test/test_dual_launch.py -q`：2 通过；`node --experimental-default-type=module --test src/nexus_viz_dashboard/web/test/localization.test.mjs`：3 通过。改动的 17 个 Python 文件通过 Flake8（沿用 E501/W503 忽略规则），`git diff --check` 通过；既有 UWB 子模块修改保持原样。

### 必须保留的失败

1. **`test_real_rosbag_roundtrip_preserves_sensor_stamps_dual_outputs_and_history` 失败**：录包数据库创建成功，但订阅发现等待超时，尚未读回图像/IMU/UWB/平台/目标/质量状态。该测试未跳过、未降低断言。
2. **旧入口的 `test_unsynchronized_sources_degrade_to_newest_source` 失败**：未在期限内得到预期的最新 UWB 降级输出。使用 `colcon test --packages-select nexus_bringup --ctest-args -R test_test_realtime_pipeline_launch.py` 定向重跑仍失败。关联旧 launch / fusion / 测试代码本轮未改动，根因尚未确认，不能据此声称旧链路回归通过。

独立诊断中，两个 ROS2 进程的晚加入话题发现出现异常；系统自带 `ros2 topic pub` 配合 `ros2 bag record` 也出现空包，而部分直接 echo 收发可成功。更换隔离 domain、localhost 设置及临时 Fast DDS 配置未解决录包测试。没有修改系统网络、防火墙或 DDS 安装。当前证据支持继续调查跨进程发现/可靠传输，尚不足以确认具体底层根因。Fast DDS 发现配置核对使用[官方说明](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/discovery/general_disc_settings.html)，未因此改动生产 DDS 参数。

下一步：先恢复同环境下系统自带 ROS2 发布与录包的回环，再原样重跑新录包测试和旧异步观测测试；随后执行真实模型与采集设备的统一启动验收。未取得回环数据前，不将第 5 项标记为 VALIDATED。

## 新定位入口

主入口是 `nexus_bringup dual_localization.launch.py`。旧 `target_localization.launch.py` 与 Gazebo 路线仍是历史基线，不作为这六项的主入口。历史源码/通用数值工具可保留旧状态维度，但不进入新静态目标链。

以下路径必须替换为已核验的本机文件，不能直接拿模板或合成测试标定标记为 LIVE：

```bash
ros2 launch nexus_bringup dual_localization.launch.py \
  platform_calibration:=/ABS/platform.yaml \
  vision_calibration:=/ABS/visual.yaml \
  reference_calibration:=/ABS/fixed_tags.yaml \
  superpoint_manifest:=/ABS/superpoint_manifest.yaml \
  detector_manifest:=/ABS/detector_manifest.yaml \
  input_mode:=LIVE run_id:=2026-09-03_E000_replace_with_registered_run \
  record:=true record_output:=/ABS/new_run_directory
```

- 相机输入为匹配 CameraInfo 的去畸变图像，默认 `/camera/image_rect`；采集与整流在上游。平台、视觉、固定 Tag 的实际相机安装变换必须一致。
- 上游必须提供采样时钟已核验的 `/nexus/fcu/imu` 和按锚点标定的 `/nexus/uwb/ranges`。UWB 是原始距离合同，不接受把厂商 x/y 位置当三维真值。可选 `telemetry_udp:=true` 仅接入已有只读 IMU/odom UDP 适配，不能替代 UWB 时间戳/锚点映射。
- `input_mode:=REPLAY` 必须有 run_id 并提供 `/clock`；SIMULATION/REPLAY 使用模拟时钟。合成标定仅在显式 `allow_test_calibration:=true` 且非 LIVE 时允许。
- 输出：`/nexus/platform/odom`、`/nexus/target/kinematics`、`/nexus/viz/localization_state`、`/nexus/viz/localization_markers`。RViz 默认加载新的 marker 配置。
- Web 默认 `http://127.0.0.1:8766/public/index.html`。已有服务占用端口时可设置 `web_port` / `rosbridge_port`；非默认桥接端口需在页面 URL 增加 `?rosbridge=ws://127.0.0.1:19091`。浏览器目标选择用于查看已有轨迹；注册仍使用 `/nexus/vision/target_requests` / `target_reference_image`。
- 录包默认关闭。开启时 run_id 和全新目录缺一不可；记录图像/CameraInfo/预览、IMU/原始 UWB、平台结果、目标前后端结果、特征/检测/请求、质量状态和展示消息。`run.json` 保存配置副本 SHA256、Git HEAD 和 dirty 标记；**dirty 标记不等于已归档未提交代码**，可复现实验还需保存对应工作树改动和外部模型资产。bag 时间是记录时间，原始采样时间保留在消息字段中。

## 范围与剩余边界

未改飞控、ROS1、厂商串口桥接、UAV 速度/高度控制；未覆盖历史规划、运行结论或既有子模块改动。真实相机标定、模型资产、跨机传输和飞行精度仍未验证。PnP/无纹理目标分支、完整联合因子图和 7+7 消融属于旧总体方案的后续工作，不伪装成本轮六项的软件验证结果。

未扩大任务范围。当前未提交 Git。
