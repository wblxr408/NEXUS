# E003: UWB Gazebo sandbox online smoke test

- Status: integration path implemented; this archive records the online smoke-test contract, not a numeric accuracy run.
- Scope: connect Gazebo odometry to the existing UWB simulation adapter, router Trilateration, ROS2 `TargetObservation`, and the Dashboard input topic.
- Related run: `2026-08-26_E003_uwb_sandbox_online_smoke`.

## Purpose

Confirm that the online sandbox path uses the same configured anchor/range model and Trilateration implementation as offline replay, without copying Ground Truth into the estimator.

## Online data path

```text
Gazebo planar move odometry
  -> /nexus/gazebo/uav/odom (nav_msgs/msg/Odometry)
  -> nexus_uwb_simulation
  -> configured anchor distances and noise
  -> AlgorithmRouter("uwb.matlab.trilateration")
  -> /nexus/uwb/target_observation (nexus_msgs/msg/TargetObservation)
  -> rosbridge Dashboard UWB store
```

The adapter uses odometry only to generate simulated ranges and to retain an evaluation reference. The router receives an observation frame containing timestamp, tag ID, anchor positions, measured ranges, and standard deviations; it does not receive a Ground Truth position.

## Configuration

The default anchor file is `simulation/configs/uwb_simulation_config.yaml`. Launch arguments expose `sigma_m` and `random_seed`; the default values are 0.0 m and 42. The anchor positions remain simulation baseline/provisional values.

## Verification boundary

The repository contains the launch and message contracts for the path. A Dashboard screenshot and a machine-readable online metric file were not archived for this run. Therefore this run must not be cited as a measured online RMSE result. The existing Dashboard/Gazebo records are linked from `run.md` for traceability.

## Reproduction command

```bash
source /opt/ros/humble/setup.bash
cd /home/li/NEXUS/code/ros2_ws
colcon build --packages-select nexus_msgs nexus_uwb_simulation nexus_bringup
source install/setup.bash
ros2 launch nexus_bringup sandbox_uwb.launch.py sigma_m:=0.0 random_seed:=42
```

In a second terminal, start the existing Dashboard launch:

```bash
source /opt/ros/humble/setup.bash
source /home/li/NEXUS/code/ros2_ws/install/setup.bash
ros2 launch nexus_viz_dashboard dashboard.launch.py
```

The online numeric evaluation remains a follow-up because the current adapter does not persist a run-level metrics file.
