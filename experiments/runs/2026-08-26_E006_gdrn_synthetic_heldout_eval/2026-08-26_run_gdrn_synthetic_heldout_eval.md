# Upstream GDRN custom synthetic held-out evaluation

## Purpose

Train an adapted upstream GDRN direct-pose model for the ten NEXUS
parameterised targets, then evaluate its model predictions on five held-out
UAV/camera views. Report three distinct quantities:

1. UWB multilateration `map -> base_link`;
2. GDRN `camera_0 -> target_link` under GT BOP ROI input;
3. composed UWB + onboard-attitude + GDRN `map -> target_link`.

This is a synthetic geometry experiment. It is neither a physical-sandbox nor
an official LM/LM-O/YCB-V reproduction result.

## Dataset partition and truth boundary

- Training: `nexus_sandbox_gdr_net_train_v01`, random seed `20260827`, 40
  camera views / 400 BOP object instances.
- Test: `nexus_sandbox_gdr_net_v03`, random seed `20260826`, 5 camera views /
  50 BOP object instances.
- The two camera-view sets have different seeds. Test RGB and BOP annotations
  are held out from training.
- Training uses BOP pose labels and `scene_gt_info.bbox_visib`. Test inference
  receives the same GT ROI condition, not an independently trained detector.
  Thus reported availability is pose availability conditional on a supplied
  ROI; it is not whole-image detection recall.
- Test `scene_camera.json` contains only camera intrinsics and depth scale.
  The visual model and fusion script never read `ground_truth/`.
- `platform_attitude.csv` is an explicit simulated onboard attitude input with
  zero additional error. UWB still obtains only position from noisy ranges.

## Model adaptation

- Upstream revision: `THU-DA-6D-Pose-Group/GDR-Net@1be9fe73292fd748087aa88d7bf987434f271ebb`.
- Launch environment: Windows `gdr_net` Conda environment, Python 3.10,
  PyTorch 2.6.0+cu124, RTX 4070 Laptop GPU (8 GiB).
- `gdr_net_numpy_compat.py` restores NumPy 2 legacy aliases process-locally;
  the upstream Git submodule remains unmodified.
- The custom loop retains upstream GDRN ResNet-18 backbone, dense-coordinate
  head and ConvPnP head. This first renderer does not yet supply dense surface
  XYZ labels, so only native direct rotation and translation losses are active.

## Commands

```bash
python3 simulation/generate_gdr_net_dataset.py \
  --config simulation/gdr_net_dataset_train_v01.yaml \
  --out data/processed/nexus_sandbox_gdr_net_train_v01

python3 simulation/generate_gdr_net_dataset.py \
  --config simulation/gdr_net_dataset_v03.yaml \
  --out data/processed/nexus_sandbox_gdr_net_v03

C:\\Users\\wblxr\\anaconda3\\envs\\gdr_net\\python.exe \
  code/analysis/vision/train_gdr_net_synthetic.py \
  --train-dataset data/processed/nexus_sandbox_gdr_net_train_v01 \
  --test-dataset data/processed/nexus_sandbox_gdr_net_v03 \
  --output-dir experiments/runs/2026-08-26_E006_gdrn_synthetic_heldout_eval/artifacts \
  --epochs 300 --batch-size 32 --learning-rate 0.001 --seed 20260828

PYTHONPATH=code/analysis python3 simulation/evaluate_uwb_dataset.py \
  --dataset data/processed/nexus_sandbox_gdr_net_v03 \
  --config simulation/gdr_net_dataset_v03.yaml \
  --output experiments/runs/2026-08-26_E006_gdrn_synthetic_heldout_eval/artifacts/metrics_uwb_base_link.json

PYTHONPATH=code/analysis python3 code/analysis/evaluation/pose_evaluate_cli.py \
  --dataset data/processed/nexus_sandbox_gdr_net_v03 \
  --predictions experiments/runs/2026-08-26_E006_gdrn_synthetic_heldout_eval/artifacts/predictions_camera_to_target.json \
  --output experiments/runs/2026-08-26_E006_gdrn_synthetic_heldout_eval/artifacts/metrics_camera_to_target.json

PYTHONPATH=code/analysis python3 simulation/fuse_uwb_visual_targets.py \
  --dataset data/processed/nexus_sandbox_gdr_net_v03 \
  --config simulation/gdr_net_dataset_v03.yaml \
  --camera-predictions experiments/runs/2026-08-26_E006_gdrn_synthetic_heldout_eval/artifacts/predictions_camera_to_target.json \
  --uwb-estimates experiments/runs/2026-08-26_E006_gdrn_synthetic_heldout_eval/artifacts/metrics_uwb_base_link_estimates.csv \
  --output experiments/runs/2026-08-26_E006_gdrn_synthetic_heldout_eval/artifacts/predictions_map_to_target.json

PYTHONPATH=code/analysis python3 code/analysis/evaluation/map_pose_evaluate_cli.py \
  --dataset data/processed/nexus_sandbox_gdr_net_v03 \
  --predictions experiments/runs/2026-08-26_E006_gdrn_synthetic_heldout_eval/artifacts/predictions_map_to_target.json \
  --output experiments/runs/2026-08-26_E006_gdrn_synthetic_heldout_eval/artifacts/metrics_map_to_target.json
```

## Result

The final checkpoint was trained on the first 12 complete, seed-`20260827`
training views (120 object instances), for 150 epochs, batch size 32 and
learning rate `0.001`. The larger 40-view runs were stopped before a checkpoint
was written and are not included below. The five seed-`20260826` test views
remain independent of those twelve training views.

| Output | Samples | 3D RMSE | P50 / P95 | Rotation RMSE | ADD-S mean / recall | Availability | Runtime mean / P95 |
|---|---:|---:|---:|---:|---:|---:|---:|
| UWB `map -> base_link` | 5 | 0.0461 m | 0.0404 / 0.0673 m | — | — | 1.00 | 3.73 / 14.08 ms |
| GDRN `camera_0 -> target_link` | 50 | 0.5021 m | 0.3948 / 0.9618 m | 92.40 deg | 0.3553 m / 0.00 | 1.00 | 21.27 / 6.72 ms |
| UWB + attitude + GDRN `map -> target_link` | 50 | 0.4975 m | 0.3906 / 0.9527 m | 92.40 deg | 0.3514 m / 0.00 | 1.00 | 25.00 / 23.20 ms |

Further final values:

- UWB horizontal RMSE: `0.0294 m`; height RMSE: `0.0355 m`; range-residual
  RMSE/P95: `0.00787 / 0.01758 m`.
- GDRN camera translation bias `(x, y, z)`: `(-0.1460, -0.0061, 0.0091) m`;
  projection mean/P95: `90.38 / 211.88 px`.
- Fused map-target translation bias `(x, y, z)`: `(0.1304, -0.0855, 0.0167) m`.
- Update rate is `2.0 Hz`; all outputs were present, so recovery time is
  `0.0 ms`. Sensor-to-output latency is unavailable in this offline replay and
  is intentionally recorded as `null`, rather than treated as runtime.

The visual and fused values **do not meet** the project’s centimetre-level
target. Reasons visible in this experiment are limited training coverage,
GT-ROI-only conditioning, flat geometry rendering and no dense XYZ surface
supervision. They are valid synthetic model results, not a claim of GDR-Net
accuracy on physical targets. In particular, the good UWB platform result does
not repair the much larger visual relative-pose error.

Artifacts (ignored from Git as experiment products) are under `artifacts/`:

```text
gdrn_synthetic_checkpoint.pth              58cbe4842fb385b0a612e686526079840ebc40ce8075a7401f7ab32ab663c659
predictions_camera_to_target.json           f40444930189cdf82f26c0322a70fb0b118e83f1dec9aa19c3111b3220dd6f73
predictions_map_to_target.json              321e61274e085d4b14b1a49f8b6fd7b9f873b0f8b383ef126c52977ff43f0751
```

Validation after result generation:

```text
python3 -m pytest code/analysis/test -q
15 passed
```
