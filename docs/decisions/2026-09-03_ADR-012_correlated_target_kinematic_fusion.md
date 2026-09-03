# ADR-012：相关目标米制状态的融合接口与预测语义

- 日期：2026-09-03。
- 状态：accepted（原任务范围内的实现选择；软件验证见逐项清单第七阶段）。
- 授权依据：用户完整双对象定位设计及“根据要求完成所有代码层的补全实现，并一一对应是否写完了”。

## 背景

新增目标前端输出 `TargetKinematicState`，包括p/v交叉协方差及未知姿态标记。既有 `fusion_localization_node` 消费旧 `TargetObservation`，要求有效四元数，仅使用位置协方差。直接转换会丢失速度和可观测性语义。

相邻SuperPoint多帧目标解共享图像、平台状态与外参误差，不能默认相互独立。重复应用独立测量Kalman更新可能把同一证据累积成过高精度。完整共享因子图仍是后续工作，不能用消息适配冒充已经完成联合后端。

## 决定

1. 增加 `target_kinematic_fusion_node`，直接消费 `/nexus/vision/target_kinematics`，发布 `/nexus/target/kinematics`，两者均使用现有 `nexus_msgs/TargetKinematicState`。旧节点与 `/nexus/target/pose` 保持原有接口。
2. 对未知交叉相关的目标估计使用协方差交集CI，保留p/v/姿态及其交叉块，未知分量不填单位四元数或零方差。测量自身的时间/frame/SI/可观测性仍必须通过检查。
3. 新质量匹配键为 `(target_id, source, position_reference, sample_timestamp_ns)`。共享质量JSON在原有字段之外增加 `source` 与 `position_reference`，旧订阅者不需改变字段读取。
4. 固定、规则、学习三种协方差模式沿用现有MLP；当前视觉目标输入使用视觉尺度。硬创新检查先于当次学习膨胀的应用，网络不回归坐标。
5. 发布 `/nexus/target/kinematic_fusion_status`（String/JSON schema_version=1），登记输入源、几何参考点、门控、CI权重、实际协方差尺度、质量匹配情况和失败原因。
6. 短时预测与新观测显式区分：`valid=false`、三个`*_observed=false`、`reason=prediction_only`，允许已有预测分量及协方差为有限值；`last_valid_sample_timestamp_ns`保持最后接受的观测时间。未知分量仍是NaN。超过预测时限位置也为NaN；身份歧义或参考点切换待确认时不外推旧目标点。
7. `static_visual.launch.py`默认接入新融合节点，可用 `enable_target_fusion` 独立关闭；新增目标可靠性模式/模型参数。另提供独立 `target_kinematic_fusion.launch.py`。没有添加平台控制命令。

## 选择依据与边界

未采用将新消息压成旧位置输入的方案，因为它丢失速度交叉协方差、要求伪造未知姿态，且旧更新假设不适合重叠窗口。CI使用已安装的NumPy/SciPy，接口能够立即消费已实现的目标几何输出。与完整联合共享因子图相比，CI通常更保守，SO(3)部分仍是局部线性近似；本决定不删除完整联合后端的原有未完成项。

本次新增的是融合节点及接口，不宣称完成自动关键点/PnP姿态生产、RViz/Web新消息消费、平台统一启动、7+7消融或真实飞行精度。接口详细说明见 `docs/architecture/2026-09-03_design_target_kinematic_fusion_interfaces.md`。
