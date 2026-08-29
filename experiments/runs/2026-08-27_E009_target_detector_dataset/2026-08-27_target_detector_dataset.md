# E009 十类无 CAD 代理目标：检测器到 GDR-Net 数据与接口实验

## 目的与边界

为十个固定沙盘目标建立可区分的参数化代理模型、轨迹隔离的检测/姿态数据，以及
外部检测器到上游 GDR-Net 的类别+bbox 接口。没有修改 GDR-Net submodule、网络结构、
损失或 PnP；YOLO 是位于其前的独立检测器。

这些模型不是实体 CAD 或实体测量真值。其尺寸、位置和形状均标记为 `confidence: C`，
只可作为当前仿真训练/评测基线；实体实验前必须替换为实测尺寸、纹理与相机标定。

## 相机依据

依据 `docs/2026-08-25_reference_interface_fields.csv`：采用已确认的 Gazebo IMX219
CameraInfo，分辨率 `3280×2464`，

```text
K = [[1978.892939, 0, 1640.5],
     [0, 1978.892939, 1232.5],
     [0, 0, 1]]
D = [0, 0, 0, 0, 0], distortion_model = plumb_bob
```

CSV 中实机 `D[]`、曝光、增益、滚动快门时序及 `base_link → camera` 外参均未确认。
本仿真对此采用零畸变、单位旋转和零平移，并在
`simulation/target_detection_dataset_v01.yaml` 中显式标记为 idealized，不能称为实机标定。

## 十个目标

目标定义位于 `simulation/target_catalog_v01.yaml`：立式条纹罐、卧式胶囊罐、锥顶筒仓、
三角料斗、六角柱、叠箱塔、L 型厂房、十字阀块、截锥塔和双筒单元。每个目标有唯一
`object_id/class_name`、颜色、参数化网格、尺度和 `target_link` 参考中心；均输出 PLY，
但 `proxy_cad=false`。

## 数据集与分割

最终供 Windows 训练使用的数据位于
`C:\Users\wblxr\nexus_target_detection_pose_v01`（不提交 Git；约 165 MB）。生成命令：

```bash
python3 simulation/generate_target_detection_dataset.py \
  --config simulation/target_detection_dataset_v01.yaml \
  --out /mnt/c/Users/wblxr/nexus_target_detection_pose_v01 \
  --yolo-assets copy
```

每帧输出 RGB、每目标二值 mask/visible mask、COCO bbox、YOLO bbox、BOP
`camera → target` 位姿、相机内参与 evaluator-only `map` 真值。按完整轨迹而非随机帧分割：

| 划分 | 轨迹 | 帧数 | 可见实例数 |
|---|---|---:|---:|
| train | west orbit / east orbit / diagonal | 108 | 1080 |
| val | figure-eight | 36 | 340 |
| test | perimeter arc | 36 | 360 |

验证轨迹中的两帧目标整体出视野，故可见实例为 340；这是自然不可见样本而不是漏标。训练、
验证、测试轨迹 ID 完全不重叠，验证器已通过。

## 外部检测器

使用已安装的 Ultralytics YOLO11n，不修改 GDR-Net：

```powershell
C:\Users\wblxr\anaconda3\envs\gdr_net\python.exe `
  \\wsl.localhost\Ubuntu-22.04\root\nexus_workspace\NEXUS\code\analysis\vision\run_yolo_detector.py train `
  --data C:\Users\wblxr\nexus_target_detection_pose_v01\detector_yolo\dataset.yaml `
  --model C:\Users\wblxr\nexus_detector_runs\yolo11n.pt `
  --epochs 30 --image-size 960 --batch-size 8 `
  --project C:\Users\wblxr\nexus_detector_runs --name target_detection_v01_retry
```

该模型在独立 test 轨迹上的 IoU@0.5 结果为：TP 360、FP 2、FN 0、precision `0.9945`、
recall `1.0000`、F1 `0.9972`、mAP50 `1.0000`、36/36 帧均有检测输出。该数值只代表平面
仿真代理外观，不能外推为实体沙盘检测性能。

检测输出 JSON 由 `run_yolo_detector.py predict` 写入，每条字段为：

```json
{"class_id": 1, "bbox_xywh_px": [x, y, width, height], "confidence": 0.93}
```

`detector_contract.py` 对输入做校验；`train_gdr_net_synthetic.py --detector-predictions`
只由该 JSON 构建推理 crop，不读取 BOP 真值 bbox。实测摘要记录
`roi_source: external detector class+bbox JSON`，证明该边界已执行。

## GDR-Net 接口烟测

将 362 条 YOLO 检测输出送入 GDR-Net 后，最终按 BOP 目标匹配获得 360/360 有效
`camera → target` 姿态输出；平均推理时间 `5.59 ms`。当前旧 checkpoint 并未在本十类
代理模型上训练，因而平移 RMSE `1.0196 m`、旋转 RMSE `114.82°`，不能用于性能达标结论。
这项运行只证明检测器→GDR-Net 输入链已打通且没有真值 ROI 回退。

## 校验

已执行：

```bash
python3 simulation/validate_target_detection_dataset.py \
  --dataset /mnt/c/Users/wblxr/nexus_target_detection_pose_v01
python3 simulation/evaluate_detector_outputs.py ...
python3 -m py_compile simulation/generate_target_detection_dataset.py \
  simulation/validate_target_detection_dataset.py \
  simulation/evaluate_detector_outputs.py \
  code/analysis/vision/detector_contract.py \
  code/analysis/vision/train_gdr_net_synthetic.py \
  code/analysis/vision/run_yolo_detector.py
```

数据结构、相机矩阵、十类别、COCO bbox、mask 文件、YOLO assets 和轨迹隔离均通过验证。
