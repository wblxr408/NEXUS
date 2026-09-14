# E049 — Dual-object Monte Carlo localization study

Status: `COMPUTED_APPROXIMATE_NOT_LIVE_VALIDATED`

This run makes 500 deterministic Monte Carlo replicates for two distinct
outputs:

1. UAV self-localization from the E042 four-anchor layout and the post-filter
   UWB standard deviations recorded in E043.
2. Sand-table target coordinates from the three E048 target estimates, with
   simulated UAV-position propagation.

The target candidate reference is **not** a validated cross-run ground-truth
coordinate: E048 records that the E030 physical-sandbox frame is not proven
aligned to `E047_live_map`.  Consequently, the target plots are labelled as a
sensitivity study and must not be reported as a measured target RMSE.

## Outputs

- `tbl_dual_object_monte_carlo_v01.csv`: all 500 simulated replicates.
- `metrics_v01.json`: descriptive simulation summaries and limitations.
- `fig_uav_self_error_ecdf_v01.png`: UAV self-localization error distribution.
- `fig_target_error_ecdf_v01.png`: target-coordinate sensitivity distribution.
- `fig_dual_object_xy_uncertainty_v01.png`: XY footprints and 95% covariance ellipses.
- `fig_noise_sensitivity_v01.png`: P95 sensitivity to UWB / vision-residual scale.

Run from repository root:

```bash
python3 experiments/runs/2026-09-09_E049_dual_object_monte_carlo/run_dual_object_monte_carlo.py
```

No inferential test is used; figures are descriptive Monte Carlo outputs.
