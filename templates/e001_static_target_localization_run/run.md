# 实验运行：YYYY-MM-DD_E001_static_target_localization_baseline

- 状态：planned / blocked / completed
- 目的：在同一时间栅格、同一 `map` 真值下比较已确认的目标观测方案。
- 设计文件：`experiments/designs/E001_static_target_localization_baseline.md`
- 代码提交：TBD
- 测量模型：direct_target / platform_relative（只能选一个）
- UWB 标签对象与消息语义：TBD
- 视觉相机、标签和输入时间：TBD
- 真值方法与不确定度：TBD
- 标定来源运行与配置 SHA256：TBD
- 输入数据外部位置、大小、SHA256：TBD
- 执行命令：TBD

## 准入检查

- [ ] `map`、单位、轴向和外参版本已冻结。
- [ ] 首版目标为 `target_0`，视觉标记为 AprilTag 36h11 / ID 0；`target_link` 是刚性安装板外向平面的几何中心。
- [ ] 实体 AprilTag 边长及 `tag_target → target_link` 外参已用量具复测并版本化。
- [ ] 真值与验证点独立于标定点。
- [ ] 采样时间和接收时间均可获得或明确标为 `not_measured`。
- [ ] 各输入的 frame、单位、协方差和有效性规则已经核对。

## 指标定义

3D RMSE、P50/P95、最大误差、有效样本数、可用率、更新频率、端到端延迟、异常计数；全部以设计文件的共同时间栅格和默认异常规则为准。

## 结果

仅完成运行后填写。必须区分实测、外部资料和未验证项。

## 证据与限制

填写运行目录内的配置、指标、图表和外部原始数据校验信息；无证据不作精度结论。
