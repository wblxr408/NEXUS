# 当前模型启动方式：本机 GPU

YOLO11 与 SuperPoint 已切换到 **WSL 内的 NVIDIA GeForce RTX 4070 Laptop GPU**。GPU 不可用时会报错，不会回退到 CPU。现有权重保持不变，当前执行文件为 CUDA TorchScript；先前的 ONNX 文件保留作历史资产。

## 已保存的位置

Ubuntu 模型根目录：`/root/nexus_workspace/NEXUS/data/processed/2026-09-05_gpu_inference_v01/`。

- `detector/detector_manifest.json` → `detector_gpu.pt`
- `superpoint/superpoint_manifest.json` → `superpoint_gpu.pt`
- `runtime/`：指向本机既有 CUDA 环境的必要包；ROS 的 NumPy 保持 1.21.5。
- `gpu_validation.json`、`cuda_trace.json`、`detector_ros.json`：验证结果。

从 NEXUS 根目录运行任意视觉命令，使用：

```bash
bash code/tools/run_with_camera_models.sh <命令及参数>
```

完整定位链路的 GPU 模型参数已固化到下面入口。**待真实标定文件齐全后**，传入原有三个标定文件即可：

```bash
bash code/tools/run_gpu_localization.sh \
  platform_calibration:=/实际路径/platform.yaml \
  vision_calibration:=/实际路径/vision.yaml \
  reference_calibration:=/实际路径/reference.yaml
```

以上“实际路径”为占位说明，当前没有生成虚假 measured 标定文件。启动域默认 20，入口不启动飞控控制。恢复棋盘格采集的流程仍按上一份相机操作说明执行；本次没有恢复采集。

复验 GPU 模型可直接运行：

```bash
OPENBLAS_NUM_THREADS=1 bash code/tools/run_with_camera_models.sh \
  python3 code/tools/verify_gpu_inference.py \
  --assets data/processed/2026-09-05_gpu_inference_v01 \
  --references data/processed/2026-09-05_camera_models_v01/parity
```

本地运行包引用如需重建，可执行 `python3 code/tools/prepare_gpu_runtime.py`。该工具保留既有依赖文件，不安装或替换系统包。

## 本次电脑测量

预热后各 30 次，FP32，输入尺寸不变：

| 范围 | YOLO11 中位数 | SuperPoint 中位数 |
|---|---:|---:|
| 模型前向加 CPU↔GPU 数据传输 | 5.41 ms | 13.04 ms |
| 包含图像预处理和模型后处理 | 9.96 ms | 17.54 ms |

这不是从相机曝光到定位输出的整链路延迟。ROS 检测验证通过，原有超时保护保留。真实目标泛化、真实内参及整链路飞行验证不由本次 GPU 测试代替。

构建、回归、首次冷启动失败及 profiler 限制详见 [E026 验证记录](../experiments/runs/2026-09-05_E026_local_gpu_inference/run.md)。
