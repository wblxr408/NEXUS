# Experiment run: 2026-08-26_E003_uwb_sandbox_online_smoke

- Purpose: archive the online Gazebo-to-UWB-Trilateration integration path.
- Design: `experiments/designs/E003_uwb_sandbox_online_smoke.md`.
- Code commit: `fe2367d6dd359db4f2660ddeaa3c47b0df9686af` (working tree contained the UWB adapter changes when archived).
- Environment: ROS 2 Humble, Gazebo Classic, `colcon`; exact OS/package freeze was not recorded.
- Input data: live Gazebo UAV odometry from `/nexus/gazebo/uav/odom`; no rosbag was archived.
- Input source: NEXUS sandbox world and planar-move route described in `docs/records/2026-08-26_record_gazebo_uav_continuous_route.md`.
- Configuration: `config/uwb_simulation_config.yaml`.
- Configuration SHA256: `8c23f710902fd52cf660cba63f774902059fd6f08dbe58b0065b77920f7597c6`.

## Commands

Build and launch the sandbox/UWB path:

```bash
source /opt/ros/humble/setup.bash
cd /home/li/NEXUS/code/ros2_ws
colcon build --packages-select nexus_msgs nexus_uwb_simulation nexus_bringup
source install/setup.bash
ros2 launch nexus_bringup sandbox_uwb.launch.py sigma_m:=0.0 random_seed:=42
```

Start Dashboard in another terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/li/NEXUS/code/ros2_ws/install/setup.bash
ros2 launch nexus_viz_dashboard dashboard.launch.py
```

## Interfaces

- Ground Truth topic: `/nexus/gazebo/uav/odom`.
- Ground Truth type: `nav_msgs/msg/Odometry`, frame `map`.
- Algorithm input: internal UWB observation frame; Ground Truth is not included.
- Dashboard UWB topic: `/nexus/uwb/target_observation`.
- Dashboard UWB type: `nexus_msgs/msg/TargetObservation` via the existing rosbridge client.

## Results and evidence

The sandbox GUI was started during development, but a screenshot and a machine-readable online metric export were not recorded for this run. Online RMSE, P50, P95, maximum error, success rate, and update rate are therefore `not measured` here. The existing Dashboard and Gazebo records are:

- `docs/records/2026-08-26_record_dashboard_sandbox_semantic_interaction.md`
- `docs/records/2026-08-26_record_gazebo_uav_continuous_route.md`

This run is an integration archive, not a numeric accuracy claim.

## Validation

- Analysis and adapter tests: `python3 -m pytest -q code/analysis/test code/ros2_ws/src/nexus_uwb_simulation/test` -> `18 passed`.
- ROS2 build: `colcon build --packages-select nexus_msgs nexus_uwb_simulation nexus_bringup` -> all three packages finished.
- ROS2 package test attempt: not clean in this environment. `nexus_bringup` had a read-only `/home/li/.ros/log` failure and a missing `apriltag_msgs` dependency; no source change was made for either external condition.

## Limitations

- The adapter currently publishes the estimate but does not persist a run-level online evaluation file.
- No Dashboard screenshot was archived.
- The anchor geometry is simulation baseline/provisional, not physical calibration.
- Numeric online metrics, latency distribution, and resource usage were not recorded.
