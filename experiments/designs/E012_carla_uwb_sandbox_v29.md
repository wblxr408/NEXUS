# E012: CARLA sandbox-v29 3D UWB reproduction

- Status: completed for the recorded CARLA run on 2026-08-29.
- Scope: reproduce the two requested UWB source projects through the NEXUS
  observation contract in the Linux CARLA environment.
- Related run: `2026-08-29_E012_carla_uwb_sandbox_v29`.
- Out of scope: ORB-SLAM2, GDR-Net, physical UWB measurements, and a full
  flight-dynamics model.

## Purpose

Validate the CARLA-to-UWB chain on the NEXUS three-dimensional sandbox map:

```text
CARLA sandbox-v29
  -> kinematic UAV actor transform
  -> noisy 3D UWB ranges
  -> six selected algorithm routes
  -> independent XYZ metrics and evidence files
```

The run is a controlled simulation experiment. It is not a physical-sandbox
accuracy result and does not establish the project's centimetre-level target.

## Source projects and routes

The active routes are limited to the two requested repositories:

- `cliansang/positioning-algorithms-for-uwb-matlab`: Trilateration,
  Multilateration, Taylor, EKF, and UKF.
- `qxiaofan/awesome-uwb-localization`: the UWB-only fixed-anchor range graph.

The first project is used through its NEXUS Python adapters. The second
project's ROS1/g2o range-graph objective is adapted to the NEXUS 3D contract
with NumPy because the original ROS1/g2o build is not part of the Ubuntu 22.04
runtime. Neither adapter accepts CARLA ground truth as an algorithm input.

## Fixed configuration

- CARLA: 0.9.16, Linux Vulkan offscreen server, RPC port 2000.
- Map: `CustomMaps/sandbox-v29/sandbox-v29` from the `gz_parts` CARLA package;
  the server's Town10 startup map is explicitly replaced and verified.
- Samples: 100 evaluation frames, 20 warmup frames, 0.1 s fixed step.
- Range noise: zero-mean Gaussian, sigma `0.05 m`, random seed `42`.
- UAV: movable non-vehicle actor `static.prop.mobile`, role name
  `nexus_uav`, kinematic path at nominal altitude `3.0 m`.
- Anchor fixture (local metres):
  `A1=(-15,-12,0.5)`, `A2=(15,-12,4.5)`,
  `A3=(15,12,0.5)`, `A4=(-15,12,4.5)`.
- Measurement model: Euclidean XYZ range (`dimensions=3`).
- Anchor coordinates are a simulation fixture, not surveyed hardware values.

## Metrics

For every valid estimate, the error is the three-dimensional Euclidean distance
to the independent CARLA actor transform. RMSE, P50, P95, maximum error,
availability, and runtime are retained. Invalid outputs remain invalid and are
not filled with ground truth.

## Acceptance

The run is accepted when the map name contains `sandbox-v29`, the UAV moves by
more than 1 m, all six routes return finite XYZ estimates for at least 95% of
frames, and observations, truth, estimates, metrics, and an image are written.

## Limitations

- `static.prop.mobile` is a CARLA kinematic proxy; it does not simulate rotor
  thrust, aerodynamic dynamics, or a flight controller.
- Ranges use a Euclidean model plus Gaussian noise; radio multipath, NLOS,
  antenna phase effects, and sensor-clock error are not simulated.
- The image is a top-down projection of a 3D scene. Projection to XY does not
  change the 3D range or metric calculation.
- Results are model-generated CARLA evidence, not physical accuracy evidence.
