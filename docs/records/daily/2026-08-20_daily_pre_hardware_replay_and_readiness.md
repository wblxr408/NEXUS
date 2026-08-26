# 日记录：2026-08-20 / 硬件到货前回放与准备材料

- 参与者：项目组 / Codex
- 环境/设备/代码版本：Ubuntu 工作区；ROS2 Humble 工作区；主仓库提交 `12bb9cd` 为本次开始时的基线；未接入真实飞控、UWB、相机或真值设备。
- 今日目标：完成硬件到货前的功能性回放、硬件首日验收准备、E001 运行模板、视觉准备与第三方模块启用门槛。

## 完成事项

1. 新增 `code/analysis/pre_hardware_replay.py` 和 `replay_cli.py`，以明确的 `test_sample` 输入实际调用项目坐标变换、固定协方差融合、单源回退和评估指标模块，并验证过期/frame/单位拒绝、异步拒绝及全无效状态。
2. 新增 8 个测试样例和对应单测；样例不属于传感器仿真或实验数据。
3. 新增首日硬件验收表、视觉输入/标记准备表、第三方模块审阅与启用门槛；首版视觉标记已通过 ADR-004 固定为 AprilTag 36h11（`target_0` / tag ID `0`）。
4. 新增 E001 正式运行目录模板；模板未包含实测数据、外参、真值或精度结论。

## 命令与证据

在仓库根目录执行：

```bash
python3 code/analysis/replay_cli.py \
  --input code/analysis/test_samples/pre_hardware_replay_cases.json \
  --output /tmp/nexus_pre_hardware_replay_report.json
python3 -m pytest code/analysis/test code/tools/test_channel_contract.py -q
```

结果：回放 8/8 个功能样例通过；其中单相机和双源样例均经过 `camera_0 → map` 坐标变换，再形成单源/融合输出；合成指标报告包含 4 个有效样本和延迟统计，明确 `is_measured_result: false`。分析/通道契约测试 `9 passed`。输出位于临时路径 `/tmp/nexus_pre_hardware_replay_report.json`，未作为交付物保留。

在 `code/ros2_ws/` 执行：

```bash
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

结果：已安装 `ros-humble-apriltag` 与 `ros-humble-apriltag-msgs` 后，工作空间完整构建通过：7 个包（项目自有 6 包加 `apriltag_ros`）完成；完整测试结果为 `29 tests, 0 errors, 0 failures, 6 skipped`。`rosbridge_suite` 保留在工作空间外的 `code/third_party/`，不作为 Humble 默认构建输入。

另在 `code/ros2_ws/` 执行：

```bash
source install/setup.bash
ros2 launch nexus_vision_localization apriltag_localization.launch.py --print-description
ros2 launch nexus_bringup target_localization.launch.py --print-description
```

结果：两个启动文件均被 ROS2 解析；视觉启动包含 `apriltag_ros/apriltag_node` 与项目 `vision_localization_node`，主启动包含该视觉启动。

## 结论（实测/预测/外部）

- **功能测试结果：** 项目自有的固定样例可验证当前接口与融合降级规则。
- **未进行实测：** 没有真实硬件输入、外参标定、真值、网络 bridge 联调或精度测量；因此没有任何定位精度、频率、延迟或硬件兼容性结论。
- **外部模块：** 仅完成版本固定与只读审阅边界，未运行第三方硬件或多相机模块。

## 阻塞与下一步

1. 等待硬件后按 `docs/2026-08-20_checklist_hardware_first_day_acceptance.md` 逐项确认 UWB 标签对象、官方消息语义、相机原始帧/时间戳/内参和实际 ROS1 topic。
2. 冻结测量模型、`map` 和真值方案后，复制 `templates/e001_static_target_localization_run/` 创建首个正式 E001 运行。
3. 只有在原始测距或多相机输入实际可用且有记录后，才启用相应第三方模块或 LNN 对比。
