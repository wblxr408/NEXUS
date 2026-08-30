# LightGlue 复现脚本

本目录是 LightGlue 在 NEXUS 沙盘上的复现代码。上游 LightGlue 源码在 `code/analysis/vision/third_party/lightglue`（git 子模块，cvg/LightGlue）。

## 依赖

Python 虚拟环境（仓库外）：`/home/e0h/nexus_workspace/lightglue_venv`
torch 2.13.0+cpu / torchvision 0.28.0+cpu / kornia 0.8.2 / opencv-python / numpy。

## 脚本

| 脚本 | 作用 |
|---|---|
| `demo.py` | 复现官方 demo（跑通 LightGlue 匹配，输出匹配结果图） |
| `match_sandbox.py` | 沙盘 RGB 图两两匹配，输出指标 CSV |
| `report_lightglue.py` | 完整指标报告生成器（按组长 13 项模板输出 markdown + CSV） |
| `viz_match.py` | 单对图匹配连线可视化（绿=内点，红=外点） |

## 运行

```bash
python report_lightglue.py \
  --rgb_dir data/processed/nexus_sandbox_gdr_net_v01/train_pbr/000001/rgb \
  --out_dir experiments/runs/2026-08-30_E007_lightglue_sandbox
```

实验记录见 `experiments/runs/2026-08-30_E007_lightglue_sandbox/2026-08-30_run_lightglue_sandbox.md`。
