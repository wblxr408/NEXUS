# GDR-Net metric simulation dataset generation

## Purpose

Generate a reproducible first-pass dataset from the parameterised sandbox YAML:
five random-but-seeded UAV/camera hover poses, ten fixed sandbox objects, RGB
images, instance masks, camera/object ground truth, simplified target meshes,
and noisy UWB ranges. This is a geometry and interface experiment, not a claim
of GDR-Net accuracy or physical-sandbox fidelity.

## Inputs and version

- Dataset config: `simulation/gdr_net_dataset.yaml`
- Scene config: `simulation/sandbox_scene.yaml`
- Generator: `simulation/generate_gdr_net_dataset.py`
- Dataset ID: `nexus_sandbox_gdr_net_v01`
- Scene mode/frame/unit: `metric_simulation` / `map` / `m`
- Random seed: `20260826`
- Targets: `NW-T01`, `NW-T02`, `NW-T03`, `NW-T04`, `NE-T01`, `NE-T02`,
  `NE-T03`, `NE-T04`, `SW-T01`, `SW-T02`; all are fixed parameterised objects.
- Target symmetry: continuous rotation about z; use ADD-S for cylinder targets.

## Command and output

```bash
python3 simulation/generate_gdr_net_dataset.py \
  --out data/processed/nexus_sandbox_gdr_net_v01
```

Result: `generated 5 frames x 10 targets` and 123 files (~716 KiB). The output
contains ten PLY models in millimetre BOP convention, five 960x720 RGB frames,
50 full/visible masks, BOP-style `scene_camera.json`, `scene_gt.json`,
`scene_gt_info.json`, `trajectory.csv` with four noisy UWB ranges per frame,
and separate `ground_truth/platform_pose.csv` / `target_pose_map.csv` files.

Checksums from this generation:

```text
dataset_manifest.json       037bcc6c940486c776f8fc99242cc83b61aa13fddec92839091d7b28a6ae9759
trajectory.csv              09db1c4e7b43278dd1db7cd4f3436299f8bdddd8c7e2308fea0f1b92315a16d3
platform_pose.csv           2f17117894c129bdd2c3d8ae95d5480d31738d888acc061703ce734a8f0b3f5a
target_pose_map.csv         72f93f7f56108739b5803a90f1a75fbc7a2ee767e91760b27cc12b03024142e0
```

## Interpretation and limitations

The image renderer is `opencv_parameterized_geometry_v01`: it uses the YAML
dimensions and flat colors, not surveyed CAD, measured textures, realistic
lighting, motion blur or physically depth-sorted occlusion. The ten objects are
therefore suitable for contract/coordinate/model-loading tests and for wiring
an eventual GDR-Net checkpoint, but not for reporting real-world recognition or
position accuracy. Ground truth is written only for the evaluator; an estimator
must use RGB, camera intrinsics and target models without reading GT files.

The dataset currently has no trained GDR-Net prediction. After a compatible
checkpoint is available, run inference on the RGB/mask/ROI inputs, convert its
dense outputs through `code/analysis/vision/gdr_net.py`, then evaluate both
`camera -> target_link` and `map -> target_link` using the metrics specified in
`docs/算法比较定义参数.md`. Keep those results in a new run directory.
