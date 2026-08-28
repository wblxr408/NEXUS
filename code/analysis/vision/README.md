# 视觉离线分析

放置标签检测、相机标定、三角测量和重投影误差分析脚本。

## GDR-Net

`third_party/gdr_net` 是上游 GDR-Net 的 Git submodule，固定提交见
`code/THIRD_PARTY_MODULES.md`。不要修改其源码；目标数据、权重和运行产物
不进入 Git。该版本的上游环境要求 CUDA 10.1/10.2、PyTorch、Detectron2、其
依赖及 BOP 数据集，完整上游测试命令为：

```bash
cd code/analysis/vision/third_party/gdr_net
sh scripts/install_deps.sh
sh core/csrc/compile.sh
./core/gdrn_modeling/test_gdrn.sh \
  configs/gdrn/lmo/a6_cPnP_AugAAETrunc_BG0.5_lmo_real_pbr0.1_40e.py 0 \
  <checkpoint.pth>
```

项目适配入口是 `gdr_net.py` 的 `run_gdr_net`，已经注册为
`vision.gdr_net`。它接收上游网络输出的稠密对象坐标图、可见性图、相机内参
和可选 ROI；输出的 `estimate` 是 `camera <- object` 的 3x4 齐次位姿前 3 行，
旋转为 `R`、平移以米为单位。该入口仅验证上游稠密几何结果到项目统一算法
契约的转换；在没有目标专属权重、CAD/尺度、内参和输入图像的情况下，它不会
合成或发布目标位姿。

```python
from algorithms import AlgorithmRouter

result = AlgorithmRouter("vision.gdr_net").run(
    coordinates=predicted_object_coordinates_hwc_m,
    visibility=predicted_visibility_hw,
    camera_matrix=camera_matrix_3x3,
    bbox_xyxy=detector_bbox_xyxy,  # optional if maps are ROI crops
)
```

### Windows Conda 冒烟路径

当前 WSL2 主机可访问 Windows Conda。为保留已有的 `used_pytorch` 环境，已在
Windows 侧创建独立的 `gdr_net` 环境；它继承 Python 3.10、PyTorch 2.6.0+cu124
和可用 GPU，且已安装 Detectron2、MMCV、Lightning、pypng 和 chardet。此环境
适用于验证 PyTorch/CUDA、下载权重和测试后续的 Windows 适配器，但**不能把它
当作 Linux 环境**：原上游仓库包含 `.sh` 启动器和 `resource` 等 Unix 依赖。

该环境当前的 NumPy 2 需要在导入上游前调用
`gdr_net_numpy_compat.apply_numpy2_compat()`；该 shim 仅在进程内恢复上游使用的
旧 NumPy 别名，不修改 Git submodule。模型模块的 Windows 冒烟命令为：

```bash
cd code/analysis/vision/third_party/gdr_net
C:\\Users\\wblxr\\anaconda3\\envs\\gdr_net\\python.exe -c "from pathlib import Path; import sys; sys.path.insert(0, str(Path.cwd().parents[2])); from vision.gdr_net_numpy_compat import apply_numpy2_compat; apply_numpy2_compat(); from core.gdrn_modeling.models import GDRN; print('gdrn_model_module_import_ok')"
```

`main_gdrn` 还会在导入时加载 LM/LM-O/YCB-V 的预定义数据集；因此运行完整入口
前仍须按上游 README 下载 BOP 数据及其单独发布的 `image_set` 文件。不能用空文件
或本项目的十个目标数据替代这些官方数据。

因此，未经适配不要在 Windows 环境运行上游 `test_gdrn.sh`。若要做无修改的
官方复现，应在独立 Linux Conda 环境安装其固定依赖；若继续 Windows 路线，则
必须为 Windows 维护一个不修改上游源码的启动适配器并对模型加载、CUDA 推理和
输出逐项记录验证。两条路径产出的稠密坐标图均通过本目录的 `run_gdr_net` 接入
NEXUS，之后才进入仿真评估。

### NEXUS 合成数据适配实验

`train_gdr_net_synthetic.py` 是不改动上游 Git submodule 的最小训练/推理适配器。
它复用上游 GDRN 的 ResNet、稠密坐标头和 ConvPnP 头，接受本项目 BOP 布局并导出
以米为单位的 `camera_0 -> target_link` JSON。它目前以 `scene_gt_info.bbox_visib`
作为 ROI 输入，因而只能报告“给定 GT ROI 条件下”的 6D 姿态，不可把可用率当作
整图检测率。完整已运行参数和持出集指标见 E006 实验记录。

若传入 `--detector-predictions <json>`，推理阶段改由外部检测器的类别和 bbox
裁剪，不读取 `scene_gt_info` 的真值框。JSON 契约由 `detector_contract.py` 定义：

```json
{
  "0": [{"class_id": 1, "bbox_xywh_px": [120.0, 80.0, 64.0, 72.0], "confidence": 0.93}]
}
```

这只是检测器到上游 GDR-Net 的输入适配，不改变 GDR-Net 网络、损失或 PnP 模块。十类
参数化目标的 COCO/YOLO 训练数据由
`simulation/generate_target_detection_dataset.py` 生成；轨迹隔离策略和真实设备优先的
相机标定依据记录在 `simulation/target_detection_dataset_v01.yaml`。
