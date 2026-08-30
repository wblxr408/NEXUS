# E010 场景化十类目标检测与 GDR-Net 外部 ROI 烟测

## 目的

在 E009 的十类无 CAD 参数化代理目标数据上，加入 `simulation/sandbox_scene.yaml` 中的
道路、工业岛、建筑、树列、交通灯、锥桶和道路标线背景。背景均为 `object_id=0`，不进入
目标检测或 6D 姿态标签。此运行验证真实设备字段优先的仿真数据、轨迹隔离，以及外部
检测器到 GDR-Net 的类别+bbox 输入边界。

模型尺寸/外观与背景均是参数化仿真代理，不是实体沙盘 CAD 或测量真值。结果不能外推为
实体沙盘性能。

## 数据与相机

- 数据目录（不入 Git）：`C:\\Users\\wblxr\\nexus_target_detection_pose_v02`，约 253 MB。
- 生成器：`simulation/generate_target_detection_dataset.py`；配置：
  `simulation/target_detection_dataset_v01.yaml`；十类定义：
  `simulation/target_catalog_v01.yaml`。
- 图像：3280×2464 RGB，确认的 Gazebo IMX219 CameraInfo：
  `K=[[1978.892939,0,1640.5],[0,1978.892939,1232.5],[0,0,1]]`，`D=[0,0,0,0,0]`。
- CSV 未确认的实机畸变、曝光、增益、滚动快门时序和 `base_link -> camera` 外参，均明确为
  理想化；当前外参是零平移和单位旋转。

完整轨迹划分：train 三条轨迹 108 帧（1080 实例）、val 一条 figure-eight 36 帧（360
实例）、test 一条 perimeter-arc 36 帧（360 实例）。验证器确认十类别、RGB、掩码、BOP
camera->target 标签、COCO/YOLO 框、内参与所有轨迹 ID 互不重叠。

## 检测器

在 Windows `gdr_net` Conda 环境，以 YOLO11n 训练 30 epochs（960 px、batch 8、seed
20260830）。Windows 从 WSL UNC 路径读取时，AMP 自检会尝试下载无关的 yolo26n 权重，且
多进程加载会重载 CUDA DLL；因此本次使用包装器的 `--disable-amp --workers 0`。这仅调整
训练运行方式，不修改检测/姿态算法。

权重（不入 Git）：
`C:\\Users\\wblxr\\nexus_detector_runs\\target_detection_v02_scene_context_final\\weights\\best.pt`。

独立 test 轨迹的 detector JSON 评测（IoU 0.5）：360 GT、361 检测、TP 360、FP 1、FN 0，
precision 0.99723，recall 1.0，F1 0.99861，mAP50 1.0，36/36 帧均有输出。该 JSON 位于本
运行关联的 E009 artifacts，且每条只含 `class_id`、`bbox_xywh_px` 与 `confidence`。

## GDR-Net 输入边界烟测

使用 E006 的旧 GDRN checkpoint，不进行训练，将上述 detector JSON 传给
`train_gdr_net_synthetic.py --detector-predictions`。其摘要在
`E009/.../artifacts/gdrn_from_detector_v02/training_summary.json` 中记录：

- `test_instance_count: 361`；
- `roi_source: external detector class+bbox JSON`；
- 平均网络推理时间：5.31 ms/检测。

这确认 GDR-Net 测试裁剪未回退到 BOP 真值框。旧 checkpoint 不属于该十类代理目标，故本
运行不报告其姿态精度；目标专属姿态训练及与 UWB/ORB 的最终比较应作为后续独立实验。

## 验证命令

```bash
python3 simulation/validate_target_detection_dataset.py \
  --dataset /mnt/c/Users/wblxr/nexus_target_detection_pose_v02

python3 simulation/evaluate_detector_outputs.py \
  --dataset /mnt/c/Users/wblxr/nexus_target_detection_pose_v02 \
  --split test \
  --detections experiments/runs/2026-08-27_E009_target_detector_dataset/artifacts/detections_test_v02.json \
  --out experiments/runs/2026-08-27_E009_target_detector_dataset/artifacts/metrics_detector_test_v02.json

python3 -m py_compile simulation/generate_target_detection_dataset.py \
  simulation/validate_target_detection_dataset.py \
  code/analysis/vision/run_yolo_detector.py
```
