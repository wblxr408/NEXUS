# E036：Tiny-SuperPoint 轻量特征学生蒸馏

## 状态

VALIDATED（RRD/CARLA 仿真图像上的特征学生蒸馏、ONNX 导出与 CPU 实际前向）。不将其
表述为实体沙盘特征泛化或边缘设备实测时延。

## 训练输入

- 教师：E026 已验证的 CUDA TorchScript SuperPoint，
  `data/processed/2026-09-05_gpu_inference_v01/superpoint/superpoint_gpu.pt`。
- 图像：E023 已核验的 RRD CARLA episode 的 500 帧 `640×480` RGB 图像。
- 学生：项目内 Tiny-SuperPoint，保持在线 `SuperPointOnnx` 所需的 65 通道检测头和
  256 维描述子头；训练 2 epoch、batch 4、CUDA FP32。教师始终冻结。

## 产物

输出目录（Git 忽略的大模型资产）：
`data/processed/2026-09-07_tiny_superpoint_rrd_distill_v01/`。

- `tiny_superpoint_state.pt`：SHA-256
  `2419455009dcb725ea0294eef6a9b51f90628a1bffac439e04764e183cc5da71`；
- `tiny_superpoint.onnx`：SHA-256
  `3181a9847db68fc878dc05c217f68f740306d191bba3703639f31561e7d3c92a`；
- `superpoint_manifest.json`：显式声明 `onnxruntime_cpu`、`640×480` 输入、
  `superpoint_mit_v1` 描述子采样和 `distillation_candidate_not_field_validated`。

## 验证

在 WSL CPU ONNX Runtime 上，以 RRD `000030.png` 进行实际前向：

- `logits`: `[1, 65, 60, 80]`；
- `descriptors`: `[1, 256, 60, 80]`；
- `image_scale=0.5` 时仍保持原始坐标口径，选出 128 个稀疏特征；
- 本机单帧前向和特征选择记录为 `130.757 ms`。这是工作站 CPU 上的单样本结果，
  不代表树莓派、ROS 端到端延迟或精度。

训练使用既有 `gdr_net` CUDA 环境和 E025 已保存的 ONNX 导出依赖，未安装或升级依赖。

## 迁移限制

该资产可作为 `lite_superpoint_manifest` 的候选。实体相机采集后必须以真实图像重新
蒸馏或至少进行跨域检查，且只有重新验证描述子匹配、身份保持和端到端延迟后才能作为
默认机载轻量模型启用。

## 动态分辨率轻量档

此前固定输入尺寸的特征模型即使接收 `image_scale`，仍会在进入网络前重采样回固定尺寸，
不能以此声称减少 MAC。为此本运行新增 `dynamic_input` manifest 契约并重新导出：

- `data/processed/2026-09-07_tiny_superpoint_rrd_distill_v01_dynamic/tiny_superpoint.onnx`，
  SHA-256 `d9beb32e912b983cba58a2680dadee420bba4798382afa38a6f116c8d2dc58d8`；
- 以同一 `640×480` RRD 图像、`image_scale=0.5` 实测，网络输出
  `[1,65,30,40]` 和 `[1,256,30,40]`，证明实际输入为 `320×240`；输出特征仍映射回
  `640×480` 相机坐标；本机 CPU 单帧为 `31.624 ms`。

`test_dynamic_superpoint_scale_changes_network_tensor_shape_without_changing_camera_coordinates`
覆盖了该输入形状与坐标不变式。该性能仍非树莓派或 ROS 端到端结果。
