# E026：本机 RTX 4070 CUDA 模型部署

状态：**VALIDATED（GPU 模型执行、数值一致性和 ROS 检测接入）**。不代表整个无人机定位项目完成。

## 环境与代码

- 用户要求禁用 CPU 模型推理，使用本机 GPU；本次未恢复棋盘格采集，也未操作飞控。
- GPU：NVIDIA GeForce RTX 4070 Laptop GPU，8 GB。WSL NVIDIA-SMI 显示 Windows driver 566.07，支持 CUDA 12.7。
- 实际 WSL 模型运行：torch 2.5.1+cu121、CUDA runtime 12.1、cuDNN 9.1、FP32，TF32 关闭。NumPy 1.21.5、OpenCV 4.10.0 仅用于预／后处理。
- CUDA 依赖复用 `/root/nexus_envs/gdr_net_packages`，项目 `runtime/` 只引用必要包，不带入其中的 NumPy 2.2.6。无驱动更新、无新增下载安装依赖。
- 导出使用已有 Windows torch 2.6.0+cu124，在 RTX 4070 上 trace，再在 WSL torch 2.5.1 上加载验证。输入固定 640×640 / 640×480；没有训练新权重或降低图像分辨率。
- 资产根目录：`data/processed/2026-09-05_gpu_inference_v01/`，由 Git 忽略。manifest 包含 SHA256、设备、精度与原始权重来源；`runtime_sources.json` 记录本机依赖引用。
- 当前工作树包含前序工作；本次代码与元数据另存 `source_snapshot/`、`source_checksums.json`、`asset_checksums.json`。原 E025 代码快照和模型证据保留。

## 正确性与 GPU 证据

- 8 张 E023 留出图像，与 E025 保存的 PyTorch 参考张量比较：检测密集输出通过 `rtol=1e-4, atol=0.002`，NMS 后类别一致、框差小于 0.01 px；每图 3 个目标。
- 两张 SuperPoint 图像的 logits / descriptors 通过 `rtol=1e-4, atol=1e-4`；特征数 570 / 571。
- 两个模型参数加载到 cuda:0；运行时检查所有模型输出张量都位于请求的 CUDA 设备，然后才拷回 NumPy。
- profiler 记录两次模型执行中的 372 次 CUDA 内核启动 API；CUDA Event 对 GPU 驻留输入计时分别约 6.26 / 11.65 ms。PyTorch 峰值 allocated 显存 184.22 MiB（不等于包含驱动缓存的整机显存占用）。
- 设置 `CUDA_VISIBLE_DEVICES=-1` 后，实际检测器初始化报 `CUDA unavailable; CPU fallback is disabled`，进程退出非零；这是预期的负向验证。
- 真实 ROS ObjectDetectionNode 使用 CUDA 模型，3 条检测消息全部 valid、每条 3 个目标。测试域 89、localhost-only；未发布飞控控制量。

## 性能

预热后各 30 次，计时前后同步 GPU；模型计时包含输入上传、推理完成和输出下载，不包含 NPZ 数据文件解压。

| 测量范围 | 中位数 ms | 95 分位 ms |
|---|---:|---:|
| YOLO11 前向与传输 | 5.412 | 6.917 |
| SuperPoint 前向与传输 | 13.044 | 16.310 |
| YOLO11 预处理、推理、后处理 | 9.960 | 12.053 |
| SuperPoint 预处理、推理、特征后处理 | 17.542 | 19.641 |

不同测量批次受显卡动态频率影响；不能将模型计时当作相机→定位端到端耗时，也不能据此承诺固定帧率。

## 发生过的失败与处理

1. 首次调用安装包未找到新增 cuda_model 模块：新模块尚未进入 ament 安装目录，构建后解决。
2. 首轮 profiler 断言要求逐内核 GPU activity，但当前 WSL 只返回 CUDA runtime/driver API，导致断言失败。未声称活动明细可用；改用内核启动 API、CUDA 输出设备、Event 计时、显存以及禁用 GPU 的负向测试交叉验证。`kernel_activity_trace_available=false` 仍保留，逐内核分析仍未验证。
3. 首轮 ROS 测试一条消息因约 939 ms 的 TorchScript 首次优化触发 `detection_inference_stale`，另两条有效。失败记录保存为 `detector_ros_before_warmup.json`。将 3 次 CUDA 预热移至模型初始化、订阅创建之前后，3 条消息全部通过。没有放宽 300 ms 超时。
4. 初版基准将 NPZ 解压包含在前向计时表达式中；已将数据解压移出计时范围，旧报告标为 `gpu_validation_before_timer_fix.json`，上表仅使用修正后的报告。

## 重跑命令与结果

```bash
python3 code/tools/prepare_gpu_runtime.py
source /opt/ros/humble/setup.bash
cd code/ros2_ws
colcon build --packages-select nexus_vision_localization --symlink-install
cd ../..
ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 \
  bash code/tools/run_with_camera_models.sh colcon --log-base /tmp/nexus_gpu_colcon_logs test \
  --base-paths code/ros2_ws/src --build-base code/ros2_ws/build --install-base code/ros2_ws/install \
  --packages-select nexus_vision_localization
colcon test-result --test-result-base code/ros2_ws/build/nexus_vision_localization --verbose
OPENBLAS_NUM_THREADS=1 bash code/tools/run_with_camera_models.sh python3 code/tools/verify_gpu_inference.py \
  --assets data/processed/2026-09-05_gpu_inference_v01 \
  --references data/processed/2026-09-05_camera_models_v01/parity
ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 \
  bash code/tools/run_with_camera_models.sh python3 code/tools/verify_detector_ros.py \
  data/processed/2026-09-05_gpu_inference_v01 \
  data/processed/2026-09-05_camera_models_v01/parity/reference_00.npz
```

构建通过；15 组 CTest 全部通过，colcon 汇总 87 checks、0 errors / failures / skipped（含套件计数）。相关 Flake8、bash 语法及任务范围 diff 检查通过。GPU 启动包装器 `--show-args` 通过。

## 范围

修改两个视觉适配器、共用 CUDA 执行模块、必要导出／验证／运行脚本和对应记录。保留前序工作、系统环境、原权重、时间映射和标定保护。棋盘格采集继续暂停，完整定位启动仍要求真实标定文件。未扩大任务范围。
