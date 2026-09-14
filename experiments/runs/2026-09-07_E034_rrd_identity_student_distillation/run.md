# E034：RRD 实例身份轻量学生蒸馏

## 状态

VALIDATED（RRD/CARLA 单 episode 身份学生训练与加载）。这不是实体沙盘模型微调，
也不证明遮挡、相似实物、不同光照或实机重识别精度。

## 输入

- RRD CARLA episode `episode_20260831T132747Z_seed20260831`，来自 E023 已核验的
  500 帧、三实例、1500 条可见框仿真数据。
- 180 帧按时间均匀抽取；每个框通过 E026 已验证的 GPU SuperPoint 教师提取 256 维
  描述子，正例为同一 `target_id` 的相邻观测，困难负例为同帧不同 `target_id`。
- 教师：`data/processed/2026-09-05_gpu_inference_v01/superpoint/superpoint_gpu.pt`。

## 产物

- 描述子对：
  `data/processed/2026-09-07_rrd_identity_student_v01_pairs.npz`，共 1,074 对，
  正/负各 537，对应 SHA-256
  `f4988d34990ce9d7964bb97625d4784d169b79e2aea4769615313ca4a85557e4`。
- 轻量身份学生：
  `data/processed/2026-09-07_rrd_identity_student_v01/identity_student.npz`，
  Q/K/V 形状均为 `256 × 32`，对应 SHA-256
  `db45efca4d39644da7768920e4f1badf70be9a416088b661e2e52afba7f0f322`。

该 checkpoint 可以直接传给 `identity_transformer_checkpoint`；在线
`TargetVisualFrontend` 已成功加载并识别其 32 维投影。

## 命令

```bash
# Windows CUDA Python；读取共享工作区而不复制原始数据
python code/analysis/vision/build_identity_descriptor_pairs.py \
  --observations <RRD>/vision_target_observations.jsonl \
  --images <RRD>/sensors/rgb_mono \
  --superpoint-manifest data/processed/2026-09-05_gpu_inference_v01/superpoint/superpoint_manifest.json \
  --output data/processed/2026-09-07_rrd_identity_student_v01_pairs.npz \
  --maximum-frames 180

python3 code/analysis/vision/train_edge_students.py identity \
  --pairs data/processed/2026-09-07_rrd_identity_student_v01_pairs.npz \
  --output data/processed/2026-09-07_rrd_identity_student_v01 \
  --label-provenance 'RRD CARLA episode; target_id labels; 180 evenly selected frames'
```

## 验证与限制

- `LightweightIdentityTransformer(checkpoint)` 与 `TargetVisualFrontend` 均成功加载
  该 checkpoint；Q/K/V 尺寸、有限性、投影维度符合在线接口。
- 训练与验证来源同一 episode，故未计算也不报告泛化指标。
- 实体沙盘参考图采集后，需以注册表稳定 ID 重新构建真实正例、相似实例负例和遮挡序列，
  并以独立采集段评估，不能以此文件替代实体域蒸馏。
