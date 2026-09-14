# E033：软件链路闭环与离线回放复核

## 状态

VALIDATED（软件与合成/录制输入）。本运行验证的是可复现的软件链路，**不**代表
真机飞行精度、真实相机参考图质量或 IMU 实测已经完成。

## 覆盖范围

- 真值地图与实例注册：沿用 E030 的 `physical_sandbox_map_v01` 和 517 个用户接受的
  运行真值实例；注册表将其映射为检测、参考图和评估使用的稳定 ID。实体参考图的
  `reference_capture` 仍为 `pending`，不会伪造为已采集。
- 模型与边缘策略：E023 的 YOLO11n 训练/评估只使用 RRD/CARLA 合成数据；E025/E026
  已验证模型转换与本机推理。Nano 蒸馏、轻量特征、ROI/KV 历史和自适应策略的代码
  通过单元测试，但尚未以实体沙盘图像微调。
- ROS 主链：相机/检测/特征/身份/目标米制输出/融合节点的单元与集成测试；录包和
  回放可在本机 DDS 配置下完成。
- 网页：参考目标注册和本地定位看板的契约测试通过，不访问飞控、不发送控制指令。

## 本次修复

1. 视觉调度会向特征前端传递 `image_scale`。测试替身未实现该参数，导致自适应
   跟踪路径在测试中被静默异常吞掉；现已让替身遵守与生产前端一致的调用契约。
2. 质量包新增的身份、离群、振动和滚动快门字段现在被可靠性特征提取接受；为保持
   既有 20 维学习模型及其权重兼容，这四项作为调度补充字段而非擅自改变学习模型维度。
3. `telemetry_udp_node` 在 ROS launch 先关闭 context 时会使 `spin()` 抛出 `RCLError`
   并以错误码退出。现将该正常关闭竞态与外部关闭信号一起处理，保证只读遥测节点
   能干净退出。

## 验证命令与结果

```bash
OPENBLAS_NUM_THREADS=1 PYTHONPATH=code/analysis \
  python3 -m pytest code/analysis/test -q
# 97 passed

cd code/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select nexus_vision_localization nexus_fusion_localization --symlink-install
# 2 packages finished
colcon build --packages-select nexus_bringup --symlink-install
# 1 package finished

source install/setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=216 OPENBLAS_NUM_THREADS=1 \
  python3 -m pytest src/nexus_vision_localization/test src/nexus_fusion_localization/test -q
# 164 passed

ROS_LOCALHOST_ONLY=0 ROS_DOMAIN_ID=230 FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA \
  OPENBLAS_NUM_THREADS=1 python3 -m pytest src/nexus_bringup/test -q
# 12 passed
```

录包回环需要本机 WSL 中 Fast DDS 的 `LARGE_DATA` 传输配置；在上述配置下
`test_dual_recording.py` 为 `2 passed`。同配置下还以官方 ROS2 talker/echo 验证过
跨进程发现与收发，故该设置不是测试替代品而是本机 DDS 的必要运行配置。

```bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=216 OPENBLAS_NUM_THREADS=1 \
  python3 code/analysis/replay/replay_cli.py \
  --input code/analysis/test_samples/pre_hardware_replay_cases.json
# passed=true, case_count=8, samples=4, measured_result=false

cd code/ros2_ws/src/nexus_viz_dashboard/web
node --test test/registration.test.mjs test/localization.test.mjs
# 11 passed
```

仪表板 ROS 契约测试为 `8 passed`；边缘模型、身份与自适应策略的定向测试为
`19 passed`。

真值展开器以 `sandbox_scene.yaml`、物理尺寸参考和红绿灯照片 survey 重建后，与仓库
提交的 `physical_sandbox_ground_truth_v01.json` 及
`physical_sandbox_model_reference_registry_v01.json` 均逐字节相同；对应 SHA-256 为
`d2f451062f7888e75d856a9562d5df47930020a523dcff59385441cba9ef3b35` 和
`3c593677e270155f3ff001be08fc510892022a42b06ff399da16109a2bb2ea7a`。

## 尚待真机且不由本运行声称完成的事项

电池充电后只需补采集和验证，不需要重写软件：

1. 用 IMX219 实体画面采集各实例的参考图，并将注册表中的 `reference_capture` 从
   `pending` 更新为已采集的文件、时间戳和相机标定版本。
2. 以实体沙盘图像和已接受真值导出训练/验证样本，执行物理域微调或 Nano 蒸馏；E023
   的仿真权重可作初始化，但不能宣称已经适配实体外观。
3. 静置、匀速、转动条件下采集 UWB/IMU/相机同步 rosbag，运行离线回放并写入独立
   实测运行记录。此前已观察到四路 UWB 原始量程，IMU 的完整实测仍待采集。
4. 在安全约束下执行实机端到端定位与主动规划验证；本运行不涉及飞控控制。

## 结论

当前的软件、真值注册、合成训练基线、离线回放、仿真/单元测试和网页展示已经形成
可复现闭环。后续真机工作的关键是采集实体数据并验证，而不是再次开发同一条软件链路。
