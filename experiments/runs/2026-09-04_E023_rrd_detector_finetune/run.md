# 实验运行：2026-09-04_E023_rrd_detector_finetune

- 目的：核验用户提供的 RRD 数据能否用于训练，并完成三类静态目标检测器微调；根据已有硬件事实补一套轨迹一致的 IMU 仿真输入。
- 数据性质：压缩包自身 `manifest.json` 声明为 CARLA RRD **仿真**，500 帧、单目 RGB、三目标、模拟 UWB；不是实机沙盘采集。
- 代码提交：工作树 HEAD `1495b00`，含未提交修改；本记录不把 dirty 工作树冒充提交。
- 输入 archive SHA256：`7759de118d35e834e3a8a03107bd5f927d55bfebc90ba5c15d9abdd27c17d052`。
- 原 episode 校验：`SHA256SUMS` 共 511 项，0 项失败；500 张 `640×480` PNG、1500 个可见框。
- 环境：Windows Python 3.10.19、PyTorch 2.6.0+cu124、Ultralytics 8.4.47、RTX 4070 Laptop GPU。没有升级或安装依赖。
- 配置：`config/detector_training.yaml`、`config/imu_assumptions.yaml`。

## 检测训练

仓库根目录 `yolo11n.pt` 经 checkpoint 元数据核验为 COCO 80 类通用预训练权重。GDR 实验目录内的微调权重没有用于本次初始化。时间顺序切分为训练 300 帧、验证 80 帧、测试 80 帧，两个边界各留 1 s，总计舍弃 40 帧；不同 split 的解码图像无完全相同 SHA256。

训练使用 30 epochs、`imgsz=640`、batch 8、seed 20260904、确定性模式、AMP 关闭。外部训练集成关闭。最佳权重 SHA256：`f0d3a67f3215ae565ed3423980061ec5ab19cf7b975e2f0fd516271711fdf333`。

独立调用最佳权重在 `test` 时间块上评价：80 张图、240 个实例；precision `0.999350`、recall `1.000000`、mAP50 `0.995000`、mAP50–95 `0.952596`。RTX 4070 上该批评价记录的推理阶段为 `3.129 ms/image`，它不是 ROS 端到端延迟。

首轮单独评价因 Ultralytics 将 `path: .` 相对运行目录解析而失败；改用训练包装器已生成的绝对路径配置后通过。没有改变图像、标签或评价阈值。可视化抽查确认预测框落在三辆目标车上。

这些数字只覆盖同一 episode、同一场景、同一天气及同三个目标实例；相邻时间帧高度相关。它们不能证明独立飞行、不同场景、实机或任意目标泛化，当前 `.pt` 仅为候选训练产物，尚未转 ONNX、接入 ROS 或完成跨域验收。

## IMU 模拟

硬件静态调试只建立了：`SCALED_IMU` 约 `188.9 Hz`；加速度线值实际按 `mm/s²`，角速度按 `mrad/s`；单条样本不足以估计偏置和噪声密度。模拟器从存储的平台位姿求速度、加速度和角速度，再以 188.9 Hz 输出理想版和假设噪声版；不生成或发送任何无人机控制量。

原平台位姿包含 5 处“相邻帧完全重复、下一帧补跳”。整段样条直接求导的峰值为 `61.041 m/s²`。生成器没有把它当成真实飞行动力学，而是排除停帧及邻近区间：4714 个时间点中 4525 有效、189 无效；有效段最大运动加速度 `0.2473 m/s²`，角速度为 0（原数据平台姿态恒定）。

陀螺初始 bias 使用单条静置读数 `[-0.006, 0.003, 0.011] rad/s` 作为粗略模拟值，但没有把它冒充样本均值；噪声密度、加速度 bias 和安装旋转没有硬件实测，因此 profile 明确标为 `ASSUMED_NOT_HARDWARE_CALIBRATED`。理想版与带噪版均不属于真实 IMU 数据。输出在 `data/processed/2026-09-04_rrd_imu_{ideal,assumed}_v01/`，每条含采样时刻、有效性、segment id 和 SI 单位。真值 sidecar 与观测分开，不能作为定位器输入或学习特征。

## 验证

```bash
OPENBLAS_NUM_THREADS=1 PYTHONPATH=code/tools python3 -m pytest code/tools/test_prepare_rrd_detector_dataset.py -q
OPENBLAS_NUM_THREADS=1 PYTHONPATH=code/analysis python3 -m pytest code/analysis/test/test_simulate_rrd_imu.py -q
python3 -m flake8 --ignore=E501,W503 code/tools/prepare_rrd_detector_dataset.py code/tools/test_prepare_rrd_detector_dataset.py code/analysis/localization/simulate_rrd_imu.py code/analysis/test/test_simulate_rrd_imu.py
git diff --check
```

数据准备 3 项测试、IMU 数值 4 项测试通过；最终 Flake8 与 diff 检查通过。模型训练和测试日志、权重、图表、转换后的检测数据集均在外部证据目录，未加入 Git。

## 证据

外部目录：`C:/Users/wblxr/.codex/visualizations/2026/09/03/01a065c8-bb9e-7752-93d1-4ef1f2ba16d1/rrd_detector_training_v01/`。关键文件为 `runs/detect/training/rrd_three_classes/weights/best.pt`、`test_metrics.json`、`train.log`、`test.log` 与 `evaluation/heldout_time_block-2/val_batch0_pred.jpg`。

状态：**VALIDATED（单 episode 仿真检测微调与轨迹派生 IMU 软件生成）**。真实 IMU 噪声、相机实拍、跨场景检测和 ROS 模型接入仍未验证。未扩大到飞控控制或实机精度声明。
