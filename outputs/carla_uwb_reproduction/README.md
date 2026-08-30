# CARLA UWB reproduction bundle

This directory contains the output of run
`2026-08-29_E012_carla_uwb_sandbox_v29` on the CARLA
`CustomMaps/sandbox-v29/sandbox-v29` 3D map. It is a simulation/test-sample
bundle, not a physical UWB dataset. The run record and SHA256 manifest are in
`experiments/runs/2026-08-29_E012_carla_uwb_sandbox_v29/`.

- `observations.csv`: algorithm-facing 3D ranges only;
- `ground_truth.csv`: independent CARLA UAV actor transform;
- `estimates.csv`: six route outputs;
- `metrics.json`: machine-readable metrics and provenance;
- `carla_uwb_overhead.png`: annotated top-down projection of the 3D scene;
- `carla_uwb_overhead.json`: visualization metadata.
