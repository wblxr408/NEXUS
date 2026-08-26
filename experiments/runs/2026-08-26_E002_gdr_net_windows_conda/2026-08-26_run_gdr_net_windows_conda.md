# GDR-Net Windows Conda 环境检查

## 目的

优先复用 Windows GPU 环境，同时不修改原有通用 Conda 环境；确认该环境是否可
作为上游 GDR-Net 的运行基础。此记录不是模型推理或精度实验。

## 环境与操作

- Windows Conda 根目录：`C:\Users\wblxr\anaconda3`
- 基础环境：`used_pytorch`
- 新建专用环境：`gdr_net`（`conda create --offline -y -n gdr_net --clone used_pytorch`）
- Python：3.10.19
- PyTorch / TorchVision：2.6.0+cu124 / 0.21.0+cu124
- GPU：`torch.cuda.is_available() == True`，CUDA runtime 12.4
- 上游源码：`GDR-Net@1be9fe73292fd748087aa88d7bf987434f271ebb`

## 已验证命令

```bash
C:\Users\wblxr\anaconda3\envs\gdr_net\python.exe -c "import torch, torchvision; print(torch.cuda.is_available())"
# True

C:\Users\wblxr\anaconda3\envs\gdr_net\python.exe -c "import core.gdrn_modeling.main_gdrn"
# ModuleNotFoundError: No module named 'setproctitle'
```

初次检查时导入在缺失 `mmcv` 处停止；当时环境还缺少 `detectron2`、
`pytorch_lightning`，也没有目标权重和数据集。

后续已在隔离环境中安装 `mmcv==1.7.2`（源码构建成功）、
`pytorch-lightning==1.6.0`（pip 24.0 + setuptools 80.10.2），再次导入已推进到
`ModuleNotFoundError: No module named 'detectron2'`。PyPI 没有 Windows Detectron2
发行包；从 GitHub 源码安装时连接被重置，尚未编译成功。

## 结论

Windows 环境已确认可使用 GPU，且不会影响原 `used_pytorch` 环境。但上游 GDR-Net
未提供 Windows 原生测试路径：启动脚本为 Shell，入口还含 Unix `resource` 模块。
不能仅因 PyTorch 可见 GPU 就宣称 GDR-Net 已在 Windows 跑通。完整无修改上游复现
仍以 Linux Conda 为准；若选择 Windows 路线，必须新增外部启动适配器并验证依赖
兼容、checkpoint 加载和实际推理。

## 仿真基线检查

```bash
python3 simulation/generate_sandbox.py \
  --scene simulation/sandbox_scene.yaml --out /tmp/nexus_gdr_sim_<temporary>
# generated 486 objects
```

生成的场景明确标记为 `scene_mode=metric_simulation`、`frame=map`、`unit=m`。
当时检查时 `simulation/target_spec.yaml` 尚未定义机外目标；随后在 E003 中已冻结
十个固定目标并生成数据集。E003 的数据仍是几何渲染基线，尚未产生 GDR-Net
预测或 6D 姿态指标。

## 仿真评测准入

最终仿真必须根据 `docs/算法比较定义参数.md`，对同一帧集和真值分别报告
`camera -> target_link` 与 `map -> target_link`：3D 平移 RMSE/P50/P95、旋转误差、
ADD 或对称目标的 ADD-S、投影/重投影误差、可用率、更新率、时延和恢复时间。
本次检查未生成图像或位姿，因而没有上述指标。

## Follow-up: dependency and dataset-gate check

The dedicated Windows environment was subsequently extended with
`detectron2==0.6` built from source, `pypng==0.20220715.0` and
`chardet==7.6.0`. The upstream import now passes the Python package stage and
stops at its documented external-data requirement:

```text
FileNotFoundError: .../datasets/BOP_DATASETS/lm/image_set/ape_train.txt
```

This is not a synthetic NEXUS dataset error and it is not resolved by creating
empty placeholder files. The upstream repository imports built-in LM/LM-O/YCB-V
dataset definitions eagerly and requires the official BOP layout plus its
separately published `image_set` files before `main_gdrn` can initialise. The
next genuine upstream smoke test therefore needs a downloaded official BOP
dataset/image-set bundle and a compatible public checkpoint. The custom NEXUS
v02 test data is ready for the later custom-data adapter, but it cannot make an
LM/YCB-V checkpoint recognise the ten sandbox targets.

The environment currently uses NumPy `2.2.6`, while this upstream revision
uses removed aliases such as `np.float` and `np.maximum_sctype`. Instead of
altering the pinned submodule, `code/analysis/vision/gdr_net_numpy_compat.py`
installs those aliases for the launch process. The following Windows-Python
model-module smoke test then passed:

```text
numpy2_compat_applied True
gdrn_model_module_import_ok
```

This confirms that the GDR-Net model definitions can import under the
dedicated Windows CUDA environment. It is deliberately not reported as a
checkpoint inference or an end-to-end `main_gdrn` result; the official BOP
data/image-set prerequisite above remains unmet.
