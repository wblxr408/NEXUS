# UWB platform self-localization baseline

## Purpose

Run the project multilateration baseline on the same five deterministic UAV
positions used by the GDR-Net metric dataset. This is a real synthetic UWB
measurement experiment and a platform (`map -> base_link`) result; it is not a
visual target or GDR-Net result.

## Command

```bash
PYTHONPATH=code/analysis python3 simulation/evaluate_uwb_dataset.py \
  --dataset data/processed/nexus_sandbox_gdr_net_v01 \
  --output /tmp/nexus_uwb_metrics_final.json
```

Inputs include four noisy ranges per frame (`sigma=0.015 m`) from the four
non-coplanar synthetic anchors in `gdr_net_dataset.yaml`. The multilateration
solver uses a linearized least-squares initialization followed by Gauss-Newton
refinement. Platform truth is read only by the evaluator from
`ground_truth/platform_pose.csv`.

## Result

```json
{
  "position_rmse_3d_m": 0.046073527161678944,
  "position_mae_3d_m": 0.04238632580430855,
  "position_p50_3d_m": 0.04037097216130016,
  "position_p95_3d_m": 0.06734304072770199,
  "position_max_3d_m": 0.07295964355140229,
  "availability": 1.0,
  "failure_rate": 0.0,
  "update_rate_hz": 2.0
}
```

These are `metric_simulation` results under the stated noise, anchor geometry,
trajectory and parameterised scene. They are not measured hardware accuracy.

## Not yet measured

No GDR-Net checkpoint has produced predictions for the ten custom objects, so
the target visual metrics (`camera -> target_link`, ADD-S, projection error) and
the fused `map -> target_link` metrics remain pending. The UWB result above is
the platform input that will be combined with those predictions once a
checkpoint is trained/loaded.
