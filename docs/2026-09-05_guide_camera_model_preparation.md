# 模型、相机标定与时间对齐入口

当前：模型部署与时间映射已验证；棋盘格采集按你的要求暂停。相机服务已停止。飞控可以保持关闭；本页所有命令均不启动飞控控制。

## 已准备的模型

资产在 Ubuntu `/root/nexus_workspace/NEXUS/data/processed/2026-09-05_camera_models_v01/`：

| 文件 | 用途 |
|---|---|
| `detector/detector_slim_manifest.json` | E023 三目标 YOLO11 检测器（引用 detector_slim.onnx） |
| `superpoint/superpoint_manifest.json` | MIT SuperPoint 特征模型（引用 superpoint.onnx） |
| `runtime/` | 独立 OpenCV 4.10，系统 OpenCV 不变 |
| `model_parity.json`、`detector_ros.json` | PyTorch／ONNX 数值比较与 ROS 检测结果 |
| `clock_probe.json`、`live_clock_ros_latest_frame.json` | 真实网络时钟与相机接入延迟记录 |

在 NEXUS 根目录，用 `bash code/tools/run_with_camera_models.sh <命令>` 运行视觉程序可加载已验证的运行库；域默认 20，可通过外部 ROS_DOMAIN_ID 显式覆盖。不要用系统 OpenCV 4.5.4 直接运行该 YOLO11 模型。

部署与导出所需版本分别固定在 `code/tools/model_runtime_requirements.txt`、`code/tools/model_export_requirements.txt`。训练权重没有改变，导出入口为 `export_rrd_onnx.py` 和 `export_superpoint_onnx.py`。导出文件均使用 SHA256 校验；原始资产和运行库不进入 Git。

## 等你说开始后再做的棋盘格采集

规格已记住：**9×6 内角点，25 mm 格长，A4 横向、100% 打印**。拍摄时核对格长，用平整硬板固定打印纸；整块棋盘格必须清晰入镜，避开机架遮挡。拍中央、四角、四边，改变远近和两个方向的倾斜，在每个姿态停稳约 2 秒。采集器只保留检测到棋盘格且与已保存姿态明显不同的帧。

树莓派 SSH 终端启动相机（只在恢复采集时执行）：

```bash
bash /home/pi/nexus_readonly_stack/run_camera_stream_v2.sh
```

Ubuntu 的 NEXUS 根目录执行（使用新目录，禁止覆盖已有原始采集）：

```bash
python3 code/tools/capture_camera_calibration.py \
  --columns 9 --rows 6 --square-mm 25 --seconds 180 \
  --output data/raw/2026-09-05_imx219_checkerboard_v01
python3 code/analysis/calibration/calibrate_intrinsics.py \
  --input data/raw/2026-09-05_imx219_checkerboard_v01 \
  --output data/processed/2026-09-05_camera_models_v01/imx219_intrinsics.json
```

如果不足 20 张有效不同视角、倾角不足、边缘覆盖不足或留出重投影误差超过 1 px，求解会失败，需要补拍。真实数据通过后才能生成 measured 内参。`preview.jpg` 只是采集反馈，不参加标定。

## 时间对齐与标定后的相机输出

相机启动后，Ubuntu 执行：

```bash
bash code/tools/run_camera_preparation.sh
```

这时使用真实传感器时间映射，输出原始图像；没有 measured 内参时不输出矫正图像。完成真实标定后再执行：

```bash
bash code/tools/run_camera_preparation.sh \
  intrinsics_file:=/root/nexus_workspace/NEXUS/data/processed/2026-09-05_camera_models_v01/imx219_intrinsics.json
```

此时才会同时发布 `/camera/image_rect` 与 `/camera/camera_info`。同步方式为 Pi CLOCK_BOOTTIME→电脑 Unix 时间的软件映射，自动刷新；不会设置系统时间。相机—IMU 时间同步和安装外参仍是单独的后续工作。

完整测试、已发生失败、性能与适用范围见 [E025 验证记录](../experiments/runs/2026-09-05_E025_camera_models_clock/run.md)。目前 YOLO11 只在 CARLA 数据上验证；SuperPoint CPU 前向约 269–369 ms，尚不能据此宣称整条定位链路实时。
