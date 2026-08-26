# E002: UWB positioning reproduction and offline comparison

- Status: completed for the recorded synthetic run on 2026-08-26.
- Scope: reproduce the five Python adapters derived from the MATLAB project and compare them on the same synthetic trajectory and range observations.
- Related run: `2026-08-26_E002_uwb_positioning_reproduction`.

## Purpose

Validate the common NEXUS data path before using physical measurements: trajectory truth -> simulated UWB ranges -> selected router algorithm -> `AlgorithmResult` -> common 2D evaluation.

This run is a synthetic model experiment. It is not a Gazebo measurement result and is not a public-dataset reproduction.

## Algorithms

- `uwb.matlab.trilateration`
- `uwb.matlab.multilateration`
- `uwb.matlab.taylor`
- `uwb.matlab.ekf`
- `uwb.matlab.ukf`

The implementations are in `code/analysis/algorithms/uwb/positioning_algorithms_for_uwb_matlab/` and are registered through `AlgorithmRouter`. The source submodule revision was `2c24c478ae0c317840fec1c29631c15dbe29a630`.

## Inputs and measurement model

The trajectory is a 100-sample synthetic Lissajous-like XY path from 0 to 10 seconds, with configured tag height 1.20 m. Four anchors are loaded from `simulation/configs/uwb_simulation_config.yaml`.

For every sample:

```text
Ground Truth position
  -> Euclidean anchor distance
  -> zero-mean Gaussian range noise
  -> UWB observation frame
  -> router-selected algorithm
```

Ground Truth is used only by the range generator and evaluation. It is not present in `UwbObservationFrame` or passed to an algorithm runner.

The anchors are a simulation baseline/provisional geometry, not physical ground truth. Anchor coordinates, noise sigma, and random seed are configuration values.

## Conditions

- Noise sigma: 0.00, 0.02, 0.05, and 0.10 m.
- Random seed: 42.
- Samples: 100 per algorithm and sigma.
- Nominal sampling interval: 0.1 s.
- Evaluation dimension: 2D XY only.
- Units: metres, runtime in milliseconds.

## Metrics

The common evaluation entry point is `code/analysis/evaluation/metrics.py`, using `position_metrics_2d` and `success_metrics`.

- RMSE and MAE are computed from horizontal Euclidean errors.
- `cep50_m` and `cep95_m` are retained field names for the 50th and 95th error percentiles; they are not a claim of a statistical CEP model.
- Maximum error and X/Y signed axis bias are reported.
- Success/failure rate uses valid outputs over all expected samples.
- Update rate and mean per-call runtime are reported where timestamps/runtime are available.

## Reproduction command

```bash
cd /home/li/NEXUS
PYTHONPATH=code/analysis:simulation python3 code/analysis/algorithms/uwb/run_comparison.py
PYTHONPATH=code/analysis:simulation python3 code/analysis/algorithms/uwb/generate_plots.py
```

The commands write the compatibility copies in `outputs/`; the run directory contains the archived copies used for this record.

## Limitations

- The anchor layout is provisional and has not been confirmed by physical calibration.
- The MATLAB source was used as the implementation reference; a full numerical equivalence sweep against MATLAB was not recorded in this run.
- The Kalman adapters use their configured constant-velocity model and initialization; their non-zero baseline error is an observed result, not a tuned target.
- This run does not measure ROS/Gazebo timing, CPU, GPU, memory, recovery time, or physical UWB behavior.
