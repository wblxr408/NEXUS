# E046 上机成功条件与故障恢复复审

日期：2026-09-08。审核结论：**BUG / 未通过“上电即可持续成功”的验收**。用户明确本轮关注能否产出结果，不要求特定精度；以下问题会导致没有坐标，不能归为可忽略的精度误差。

E045 的结论仅适用于正常输入的本地软件测试：冷启动节点可启动、静置初始化正常路径通过、目标处理段使用真实模型与已知合成平台位姿得到坐标。本轮增加重启、缺口及低频位姿场景，不将前述通过外推为所有实机条件必然通过。

## 已复现的问题

### P1：平台节点独立重启后不再初始化

测试先经实际 DDS 完成原始 IMU/测距→200 组统计→平台初始化，确认初次 Odometry 正确。然后只销毁并重新创建 PlatformLocalizationNode，保留传感器链路和初始化节点；再发送 205 组新 IMU/测距。新平台始终没有状态，等待 5 s 超时。

命令：

```bash
source /opt/ros/humble/setup.bash
source code/ros2_ws/install/setup.bash
export PYTHONPATH="$PWD/code/ros2_ws/src/nexus_bringup/test:$PYTHONPATH"
ROS_DOMAIN_ID=232 OPENBLAS_NUM_THREADS=1 FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA \
 python3 -m pytest -q experiments/runs/2026-09-08_E046_power_on_reaudit/test_restart_recovery_probe.py
```

结果：**1 failed，退出码 1**；失败断言是重启后的 platform.estimator.states 应恢复为非空。见 restart_recovery_probe.txt。

原因已确认：approximate_initial_pose_node.py 的 platform_status 在收到首次 initialized/valid 后将 initialized 永久置 true；ranges 回调随后直接返回，没有检测平台实例变化或重新初始化请求。启动入口也未配置节点故障后的完整重启策略。这个复现是合成输入上的实际 DDS 故障测试，不是实机故障注入。

### P1：IMU 缺口后当前无标记入口缺少恢复途径

使用实际 PlatformWindow：先验证正常位姿更新为 valid=true，然后插入 500 ms IMU 缺口。新数据恢复后分别在 1.8、2.0、2.5、3.0 s 尝试位姿更新，四次均 valid=false，原因均为 `IMU gap exceeds maximum_gap_s`。状态保留在缺口之前，新的约束仍需要跨过同一缺口。

拒绝跨缺口积分本身正确；系统级缺陷是当前 fixed_reference=false 的入口没有自动重新建立完整状态的恢复流程，初始化节点又已经停止发送初始位姿。E042 实际运动数据曾有 500 ms 缺口，因此该场景不是仅有理论可能性。不能通过放宽最大缺口或填补虚假 IMU 修复。

### P1（条件触发）：只有低频平台位姿时目标几何可能持续没有结果

给实际 PoseTimeline 输入两个间隔 1 s 的正确平台位姿，再查询其间图像时刻，结果为 `platform_pose_gap_too_large`。默认最大插值间隔为 0.25 s；TargetMetricNode 默认观测有效期为 300 ms。

平台节点不按每个 IMU 样本发布独立高频预测 odom，只在接受约束并求解有效后发布。正常视觉相对运动可提供较密约束，但在弱纹理/相对运动暂时失败、仅剩约 1 Hz UWB 时，目标节点没有保证满足时序要求的位姿供应。E045 GPU 目标测试提供的是图像同时间的已知位姿，因此没有覆盖这个接缝。不是断言正常视觉条件下必然失败，而是证明不能保证任意现场条件都有输出。

后两项复现命令：

```bash
source /opt/ros/humble/setup.bash
source code/ros2_ws/install/setup.bash
OPENBLAS_NUM_THREADS=1 python3 experiments/runs/2026-09-08_E046_power_on_reaudit/check_timing_recovery.py
```

脚本成功执行并记录受测实现的失败状态，见 check_timing_recovery.json；“脚本退出成功”不代表功能验收通过。

## 上电前还有的部署条件

- Pi 相机服务自启动模板尚未安装，用户已确认无人机没电。地面 launch 会重连相机 TCP，但不能凭地面启动命令自动启动 Pi 相机或飞控容器。
- 需要确认 Pi ROS 域、网络发现和源时钟与地面环境对应。IMU relay 保留源时间戳；不能假定接收成功就通过跨设备新鲜度检查。
- 单目目标需可见纹理及几何基线。近似标定可以降低标定要求，但不能让不可见目标或无视差图像必然产生可靠的米制坐标。

## 后续修复目标

1. 初始化与平台状态握手增加可辨别的重启/请求初始化流程，并在安全静置条件满足时重新建立状态；不能每个状态错误都反复重置。
2. IMU 缺口恢复应显式重新定位/初始化，不伪造缺失样本；增加断流恢复的端到端测试。
3. 为目标图像提供有明确时间和有效状态的高频平台位姿，验证真实采样率与异步到达；避免只把 200 s 均值或旧坐标重新打时间戳。
4. 上电后确认设备服务自启动，至少一次完整真实目标流程与一次短暂断连恢复，才能将运行可用性升级为实机通过。

本轮仅新增审核记录、故障复现脚本与结果，未修改生产代码或历史成功测试断言。当前存在明确未解决 BUG；不声称必定成功。未扩大任务范围。
