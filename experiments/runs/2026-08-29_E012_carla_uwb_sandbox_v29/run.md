# Experiment run: 2026-08-29_E012_carla_uwb_sandbox_v29

- Purpose: run the two requested UWB reproductions on measurements generated
  by a moving UAV actor in the CARLA `sandbox-v29` 3D map.
- Design: `experiments/designs/E012_carla_uwb_sandbox_v29.md`.
- Implementation commits: `60e0a2b` (`feat(uwb): run 3D CARLA sandbox-v29 reproduction`)
  and `4fd22fc` (`fix(uwb): normalize CARLA CSV artifacts`).
- Execution time: 2026-08-29 23:47-23:54 UTC+08:00.
- Environment: Linux container; NVIDIA GeForce RTX 3080 Ti, driver 580.76.05;
  CARLA 0.9.16; client Python 3.12.3; CARLA wheel 0.9.16; NumPy 2.5.2;
  Vulkan offscreen server on display `:8`.
- CARLA build ID: `294096eb1-dirty` for both client and server.
- Input package: `/root/autodl-tmp/gz_parts/CARLA_294096eb1-dirty.tar.gz`
  (297 parts, 7,422,239,565 bytes); SHA256
  `739688863a43993334877f28d5be8af90197cbb13c50df3c4e5356dc657b7c35`.
- Map assets: `/root/autodl-tmp/carla_linux/CarlaUE4/Content/CustomMaps/sandbox-v29/`.
  The assets are external to Git and their hashes are listed in
  `artifacts/README.md`.
- MATLAB reference revision:
  `2c24c478ae0c317840fec1c29631c15dbe29a630`.
- awesome-uwb-localization reference revision:
  `b3cd36e5d78da4daea2f350cfebeb6b8b8378810`.
- Configuration: `config/uwb_simulation_config.yaml` and
  `config/carla_run_parameters.txt`.
- Configuration SHA256: `9a900ca6e5586834039e1c7b33df771b66fb8d39262412f0af6d763a56eab120`.

## Commands

CARLA server:

```bash
cd /root/autodl-tmp/NEXUS
CARLA_ROOT=/root/autodl-tmp/carla_linux \
NVIDIA_LIB_DIR=/root/autodl-tmp/nvidia-vendor-580 \
NVIDIA_ICD=/root/autodl-tmp/nvidia_icd_580.json \
CARLA_DISPLAY=:8 \
simulation/carla/run_carla_server_linux.sh
```

Smoke test:

```bash
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 \
/root/autodl-tmp/carla-venv/bin/python \
simulation/carla_smoke_test.py --host 127.0.0.1 --port 2000
```

UWB reproduction:

```bash
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 \
/root/autodl-tmp/carla-venv/bin/python \
simulation/run_carla_uwb_reproduction.py \
  --host 127.0.0.1 --port 2000 --frames 100 --sigma-m 0.05 \
  --output-dir outputs/carla_uwb_reproduction
```

Visualization:

```bash
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 \
/root/autodl-tmp/carla-venv/bin/python \
simulation/visualize_carla_uwb.py \
  --host 127.0.0.1 --port 2000 --frames 100 --camera-height-m 48 \
  --output-dir outputs/carla_uwb_reproduction
```

## Metrics

The run contains 100 frames, 400 algorithm-facing range rows, 100 independent
truth rows, and 600 estimate rows. All routes use non-coplanar 3D anchors and
return XYZ estimates. `ground_truth_used_by_algorithm` is `false` in the
machine-readable result.

| Algorithm | Valid | 3D RMSE (m) | P95 (m) |
|---|---:|---:|---:|
| Trilateration | 100/100 | 0.263244 | 0.524537 |
| Multilateration | 100/100 | 0.263244 | 0.524537 |
| Taylor | 100/100 | 0.388635 | 0.322445 |
| EKF | 100/100 | 0.587832 | 0.932818 |
| UKF | 100/100 | 0.456199 | 0.690495 |
| awesome UWB range graph | 100/100 | 0.119220 | 0.213663 |

The CARLA UAV path length was `9.653 m`.

## Evidence

The canonical output bundle is `outputs/carla_uwb_reproduction/`; its file
hashes and map asset hashes are in `artifacts/README.md`. The bundle contains
the algorithm-facing observations, independent actor truth, all estimates,
metrics, and the annotated CARLA image.

## Validation

- CARLA client/server version check: passed.
- CARLA smoke test: `SMOKE_TEST_OK`.
- CARLA UWB runner: `CARLA_UWB_REPRO_OK`.
- CARLA visualization: `CARLA_UWB_OVERHEAD_OK`.
- UWB-focused pytest selection: `34 passed`.
- Python compile check: passed.
- ROS2 build for `nexus_msgs nexus_uwb_simulation nexus_bringup`: passed.
- ROS2 contract tests for `nexus_msgs` and `nexus_bringup`: passed.
- Full realtime ROS2 launch test: not passed in this container because the
  launch uses missing `cv2` and the Conda Python 3.12 interpreter cannot load
  the ROS2 Humble Python 3.10 `rclpy` extension. This is recorded as an
  environment limitation, not as a CARLA UWB failure.
- `git diff --check`: passed before commit.

## Conclusion and limitations

This run demonstrates a reproducible CARLA Linux -> 3D UWB observation -> six
algorithm -> XYZ metrics chain on the requested `sandbox-v29` map. It is a
simulation/test-sample result, not a physical-sandbox or final project accuracy
claim. The adapters reproduce the selected algorithm cores through the NEXUS
interface; native MATLAB execution and the original ROS1/g2o build were not
used. ORB-SLAM2 and GDR-Net are intentionally outside this run.
