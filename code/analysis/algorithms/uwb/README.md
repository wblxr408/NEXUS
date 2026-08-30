# NEXUS UWB algorithms

This directory contains the NEXUS adapters used by offline comparison and the ROS2 sandbox adapter. Algorithms return the shared `AlgorithmResult` contract and are selected through `AlgorithmRouter`.

## Implemented routes

- `uwb.matlab.trilateration` - 3D trilateration adapter based on `positioning-algorithms-for-uwb-matlab`.
- `uwb.matlab.multilateration` - 3D least-squares multilateration adapter based on the same source.
- `uwb.matlab.taylor` - 3D Taylor-series adapter based on the same source.
- `uwb.matlab.ekf` - 3D constant-velocity EKF adapter based on the same source.
- `uwb.matlab.ukf` - 3D constant-velocity UKF adapter based on the same source.
- `uwb.awesome_uwb` - UWB-only sliding range graph adapted from the upstream
  ROS1/G2O implementation.

The MATLAB reference is initialized as the submodule at `code/analysis/uwb/third_party/positioning_algorithms_for_uwb_matlab/`. Upstream source is not modified by the adapters.

The `awesome-uwb-localization` adapter reproduces only that repository's
range-graph localization core. It intentionally excludes the optional visual,
IMU, lidar, and relative-localization paths.

The active CARLA path uses four non-coplanar anchors and returns an XYZ estimate
for every route. A planar anchor layout is rejected rather than silently
reported as a 3D solution.

## Common input and routing

The algorithm-facing input is one observation frame containing `timestamp_ns`,
`tag_id`, and range samples with `anchor_id`, `anchor_position_m` (XYZ metres),
`range_m` (3D Euclidean range), and `stddev_m`. Ground Truth is intentionally
absent from this input.

```python
import sys
sys.path.insert(0, "code/analysis")

from algorithms.router import AlgorithmRouter, register_default_algorithms

register_default_algorithms()
result = AlgorithmRouter("uwb.matlab.trilateration").run(
    observation_frame=observation_frame,
)
```

## Offline comparison

```bash
PYTHONPATH=code/analysis:simulation python3 code/analysis/algorithms/uwb/run_comparison.py
PYTHONPATH=code/analysis:simulation python3 code/analysis/algorithms/uwb/generate_plots.py
```

Results are written to `outputs/tables/` and `outputs/figures/`; archived copies are kept under `experiments/runs/`.

## Sandbox online path

```bash
source /opt/ros/humble/setup.bash
cd code/ros2_ws
colcon build --packages-select nexus_msgs nexus_uwb_simulation nexus_bringup
source install/setup.bash
ros2 launch nexus_bringup sandbox_uwb.launch.py
```

The adapter subscribes to Gazebo odometry, generates ranges using `simulation/configs/uwb_simulation_config.yaml`, calls the router, and publishes `nexus_msgs/msg/TargetObservation` on `/nexus/uwb/target_observation`.

## Evaluation and limits

`position_metrics` evaluates full XYZ error and labels the result `3D`;
`position_metrics_2d` remains available for legacy horizontal-only datasets.
The CARLA experiment reports 3D accuracy against the independent CARLA actor
transform. Its anchor coordinates are a controlled simulation fixture, not a
survey of physical hardware, and the numerical equivalence sweep against
MATLAB is not part of the measured experiment.

## CARLA sandbox-v29 path

Run the Linux CARLA server first, then execute:

```bash
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 \
  /root/autodl-tmp/carla-venv/bin/python \
  simulation/run_carla_uwb_reproduction.py \
  --host 127.0.0.1 --port 2000 --frames 100 --sigma-m 0.05
```

The runner loads `CustomMaps/sandbox-v29/sandbox-v29`, moves a kinematic UAV
actor in the 3D map, generates noisy ranges, and evaluates only the six routes
listed above. The actor transform is written separately to
`ground_truth.csv`; it is never passed to an algorithm. The companion
`simulation/visualize_carla_uwb.py` writes an annotated top-down projection of
the 3D scene to `outputs/carla_uwb_reproduction/carla_uwb_overhead.png`.

## 实体沙盘离线入口

现场日志先转换为统一的“一行一个基站测量” CSV。距离单位必须按设备接口明确传入：

```bash
PYTHONPATH=code/analysis:simulation python3 code/analysis/algorithms/uwb/convert_real_data.py \
  --input /external/uwb/raw.csv \
  --anchors experiments/runs/2026-08-28_E004_uwb_real_sandbox_static/config/anchors.yaml \
  --range-unit mm \
  --output data/processed/2026-08-28_uwb_sandbox_static_v01/observations.csv
```

完成真值表后，旧的实体日志入口用同一份输入依次运行五个 Python 算法：

```bash
PYTHONPATH=code/analysis:simulation python3 code/analysis/algorithms/uwb/run_real_data.py \
  --input data/processed/2026-08-28_uwb_sandbox_static_v01/observations.csv \
  --truth data/processed/2026-08-28_uwb_sandbox_static_v01/truth_points.csv \
  --config experiments/runs/2026-08-28_E004_uwb_real_sandbox_static/config/algorithm_config.yaml \
  --output experiments/runs/2026-08-28_E004_uwb_real_sandbox_static/metrics/
```

该实体日志入口仍面向历史水平 2D 数据；入口保留缺失基站和无效算法输出，在
`results.csv` 中记录 `valid` 与 `error_reason`。没有完整三维真值和现场坐标时，
输出只能作为功能或数据质量检查，不能替代 CARLA 的 3D 验收。
