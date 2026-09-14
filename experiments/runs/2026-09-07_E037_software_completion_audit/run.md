# E037：目标交付逐项完成度审计

## 结论

软件、仿真与离线交付已闭环；实体数据采集与实体域验证尚未完成。不能将前者替代后者。

| 用户交付项 | 当前证据 | 状态 | 未完成边界 |
| --- | --- | --- | --- |
| 真值地图/实例注册表 | E030；`physical_sandbox_map_v01`、517 个稳定实例；重建哈希一致 | VALIDATED（项目运行真值） | 独立量尺 survey 可作未来复核，但用户已接受当前版本为运行真值 |
| 模型训练或微调 | E023：YOLO11n 三类 RRD 微调；E025/E026 模型转换/推理 | VALIDATED（单 episode 仿真） | 未以实体沙盘图像微调 |
| Nano/轻量蒸馏 | E034：32 维身份 Q/K/V 学生；E036：Tiny-SuperPoint ONNX、动态 320×240 实测前向 | VALIDATED（仿真训练与本机运行） | 未在树莓派或实体图像上验收 |
| 参考图管理 | E027 网页/ROS 注册；E035 可追溯资产登记和 revision 工具 | VALIDATED（软件） | 517 个实例均未采集真实参考图，当前均为 `pending` |
| ROS 离线回放 | E033：录包回环、8 案例离线回放；完整 bringup 12 passed | VALIDATED（合成/录制输入） | 尚无完整实体同步 rosbag 回放 |
| 仿真测试 | E023 RRD；E033 分析测试 97 passed，后续扩展后为 102 passed | VALIDATED | 不替代实体定位精度 |
| 算法单元/集成测试 | E033 视觉/融合 164 passed；E036 变更后 165 passed | VALIDATED | 不证明真实传感器时序或飞行扰动 |
| 网页展示 | E027 浏览器 + ROS + CUDA SuperPoint 真实链路；Node 契约 11 passed、仪表板 ROS 契约 8 passed | VALIDATED（合成输入） | 等实体相机流接入后的展示复验 |

## 当前唯一的外部前置条件

无人机/相机恢复供电后，按 E035 采集实体参考图并登记；再采集静置、移动与转动条件下
UWB、IMU、图像、CameraInfo 的同步 rosbag。该输入是实体参考图、实体微调和实体离线
回放三项共同前置；没有它不能合法把仿真/软件证据升级为实机结论。

## 本审计验证

- 读取 E030 注册表：`instance_count=517`、`ground_truth_status=operational_ground_truth_user_accepted`；
  `reference_capture` 状态为 `pending=517`。
- 核验 E034 identity 学生、E036 static/dynamic Tiny ONNX 文件存在且哈希与各运行记录一致。
- 本轮源代码检查与对应测试记录参见 E033–E036；不重新把已经通过的同一测试作为新实验。
