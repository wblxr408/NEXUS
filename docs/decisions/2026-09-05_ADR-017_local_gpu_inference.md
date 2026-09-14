# ADR-017：本机 GPU 推理

授权：用户明确要求“不要用 CPU 推理，使用本机的 GPU”。这取代 ADR-016 的 CPU 模型执行选择；历史 ONNX 与 CPU 测量保留为 E025 证据。

## 实现决定

- 直接在 WSL 中使用本机 RTX 4070 Laptop GPU，不增加 Windows 推理服务或跨系统网络接口。
- 复用已安装的 WSL torch 2.5.1+cu121、CUDA 12.1、cuDNN 9.1。项目运行目录只引用必要包，避免把该旧训练环境的 NumPy 2.2.6 带入 ROS；ROS 继续使用已验证的 NumPy 1.21.5。没有重新安装驱动或升级系统依赖。
- 将现有 E023 YOLO11 权重与 MIT SuperPoint 权重导出为固定输入尺寸的 CUDA TorchScript：YOLO 640×640、SuperPoint 640×480，FP32、关闭 TF32。未重训、未缩小输入图像、未修改类别。原 ONNX 文件留档。
- 两个已有 Python 适配器保留公开类名，内部统一通过 `CudaModel` 执行 CUDA 模型；manifest 必须声明 `model_format: torchscript`、`inference_device: cuda:0`、`precision: float32`。不再在这些适配器中创建 OpenCV CPU DNN。GPU 缺失／格式不符时明确失败，不回退 CPU。
- 模型加载时先在 CUDA 上预热 3 次，完成 TorchScript 首次优化后再允许 ROS 节点订阅图像。保留原有 300 ms 过期检查。
- 图像解码、缩放、NMS、特征后处理与 ROS 消息处理仍在 CPU；“GPU 推理”指神经网络模型计算。
- `run_with_camera_models.sh` 持久加载上述运行环境；`run_gpu_localization.sh` 为原定位入口固定两份 GPU manifest，默认域仍为 20。真实标定前置不变。

选择现有 PyTorch CUDA 的原因：WSL 已能访问 RTX 4070，已有 CUDA 依赖且实际加载成功。另装 ONNX Runtime GPU 并非完成请求所必需；拆出 Windows GPU 服务则会增加接口与部署范围。

## 验证边界

E026 的数值比较、CUDA 输出设备检查、CUDA 内核启动 API、CUDA Event 计时、隐藏 GPU 的失败测试与真实 ROS 检测测试共同验证 GPU 执行。WSL profiler 未提供逐内核 GPU activity 明细，该分析限制保留在报告中，不将 CUDA API trace 冒充逐内核时间线。

详细命令与结果见 [E026](../../experiments/runs/2026-09-05_E026_local_gpu_inference/run.md)。棋盘格采集保持暂停，飞控与相机—IMU 联合标定未涉及。
