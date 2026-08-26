# NEXUS UWB algorithms

This directory contains the NEXUS adapters used by offline comparison and the ROS2 sandbox adapter. Algorithms return the shared `AlgorithmResult` contract and are selected through `AlgorithmRouter`.

## Implemented routes

- `uwb.matlab.trilateration` - 2D trilateration adapter based on `positioning-algorithms-for-uwb-matlab`.
- `uwb.matlab.multilateration` - 2D least-squares multilateration adapter based on the same source.
- `uwb.matlab.taylor` - Taylor-series adapter based on the same source.
- `uwb.matlab.ekf` - EKF adapter based on the same source.
- `uwb.matlab.ukf` - UKF adapter based on the same source.
- `uwb.awesome_uwb` - reserved route; the G2O implementation is not connected yet.

The MATLAB reference is initialized as the submodule at `code/analysis/uwb/third_party/positioning_algorithms_for_uwb_matlab/`. Upstream source is not modified by the adapters.

## Common input and routing

The algorithm-facing input is one observation frame containing `timestamp_ns`, `tag_id`, and range samples with `anchor_id`, `anchor_position_m`, `range_m`, and `stddev_m`. Ground Truth is intentionally absent from this input.

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

Offline 2D metrics are implemented in `code/analysis/evaluation/metrics.py`, especially `position_metrics_2d` and `success_metrics`. Current runs do not claim 3D accuracy. Anchor coordinates are simulation baseline/provisional, the MATLAB numerical equivalence sweep is not fully archived, and online run-level metric persistence is not yet implemented.
