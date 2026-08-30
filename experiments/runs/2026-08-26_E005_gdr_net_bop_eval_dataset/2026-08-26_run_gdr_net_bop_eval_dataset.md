# GDR-Net BOP evaluation dataset with isolated map truth

## Purpose

Create an evaluation-oriented successor to E003 that preserves the same five
deterministic UAV hover poses and ten fixed parameterised sandbox targets, but
keeps map-pose truth out of the estimator-visible BOP camera metadata. This is
a reproducible metric-simulation dataset record, not a GDR-Net accuracy run.

## Inputs and coordinate conventions

- Generator: `simulation/generate_gdr_net_dataset.py`
- Dataset configuration: `simulation/gdr_net_dataset_v02.yaml`
- Scene configuration: `simulation/sandbox_scene.yaml`
- Dataset ID: `nexus_sandbox_gdr_net_v02`
- Random seed: `20260826`
- Evaluation split: `test/000001`
- BOP object-to-camera translation: mm (`cam_t_m2c`)
- Project prediction translation: m (`t_m`)
- `base_R_map` maps a map-frame vector into `base_link`; because the simulated
  `base_link -> camera_0` transform is identity, it is also the camera view
  rotation for this first pass.
- `map_R_target` maps target-local coordinates into `map`. All generated
  assets are aligned with map, so it is identity in this version.

## Command

```bash
python3 simulation/generate_gdr_net_dataset.py \
  --config simulation/gdr_net_dataset_v02.yaml \
  --out data/processed/nexus_sandbox_gdr_net_v02

python3 -m pytest code/analysis/test -q
```

## Result

- Generation result: `5` RGB frames, `50` object annotations, `50` full and
  `50` visible masks, `10` PLY models and `20` noisy UWB ranges.
- `test/000001/scene_camera.json` contains only `cam_K` and `depth_scale`.
  It contains no `map` position, attitude or timestamp.
- `ground_truth/platform_pose.csv` includes map translation and the 3x3
  `base_R_map` rotation; `ground_truth/target_pose_map.csv` includes map
  translation and `map_R_target` for every target/sample pair.
- Validation: `14 passed` with `python3 -m pytest code/analysis/test -q`.

Checksums:

```text
dataset_manifest.json  ff340ce39a07b5cfb938e10a0d8738c32c47bce3b626f3d9a189c3a0cc6ddd38
trajectory.csv          09db1c4e7b43278dd1db7cd4f3436299f8bdddd8c7e2308fea0f1b92315a16d3
platform_pose.csv       155f6e27bb3781977dd94a280a6b94048a0a821e79cdaa78f0e69e3d99e26d31
target_pose_map.csv     fb7e26acf09db16c6f41ce8f158ab056e681eaed295e0f3f3b00b07090e9b9cb
```

## Interpretation and limits

The renderer is OpenCV geometry with parameterised scene YAML, flat colours,
no measured textures, physical illumination, full depth ordering, sensor
noise or as-built CAD. The output can validate data layout, time/coordinate
contracts, UWB input handling and a future GDR-Net prediction conversion. It
does not train the custom ten-object model and does not establish a visual or
fusion accuracy result. In particular, no oracle pose generated from
`scene_gt.json` may be recorded as GDR-Net output.
