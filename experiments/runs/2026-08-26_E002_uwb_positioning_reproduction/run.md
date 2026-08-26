# Experiment run: 2026-08-26_E002_uwb_positioning_reproduction

- Purpose: run the common offline UWB comparison on one synthetic trajectory with four configured noise levels.
- Design: `experiments/designs/E002_uwb_positioning_reproduction.md`.
- Code commit: `fe2367d6dd359db4f2660ddeaa3c47b0df9686af` (working tree contained the UWB implementation changes when this run was archived).
- Environment: Linux, Python 3.10, NumPy, PyYAML, Matplotlib; exact package freeze was not recorded.
- Input data: generated in memory from the synthetic trajectory in `run_comparison.py`; no external dataset or rosbag.
- Input source/version: NEXUS working tree at the commit above; MATLAB reference submodule `2c24c478ae0c317840fec1c29631c15dbe29a630`.
- Configuration: `config/uwb_simulation_config.yaml`.
- Configuration SHA256: `8c23f710902fd52cf660cba63f774902059fd6f08dbe58b0065b77920f7597c6`.

## Commands

```bash
cd /home/li/NEXUS
PYTHONPATH=code/analysis:simulation python3 code/analysis/algorithms/uwb/run_comparison.py
PYTHONPATH=code/analysis:simulation python3 code/analysis/algorithms/uwb/generate_plots.py
```

## Metrics

The run contains 100 samples for each of five algorithms at each of sigma = 0.00, 0.02, 0.05, and 0.10 m. Evaluation is 2D XY in metres. RMSE, MAE, P50, P95, maximum error, X/Y axis bias, success/failure rate, update rate, and mean runtime are recorded in `metrics/metrics.json` and `metrics/comparison.csv`.

P50/P95 use the existing `cep50_m`/`cep95_m` field names. They are error percentiles, not a 3D or statistical-CEP claim.

## Results

- At sigma = 0.00 m, Trilateration RMSE is `1.3807889774712256e-15 m`, with 100/100 valid samples.
- At sigma = 0.05 m, consult `metrics/comparison.csv`; all five adapters are present with the same 100 expected samples.
- The complete result set for all four noise levels is in `metrics/metrics.json`.

These are synthetic-model results. They are not physical sandbox or public-dataset measurements.

## Evidence

- `artifacts/trajectory_sigma_0.00.png`
- `artifacts/trajectory_sigma_0.05.png`
- `artifacts/error_sigma_0.00.png`
- `artifacts/error_sigma_0.05.png`
- Original sources remain in `outputs/figures/` and `outputs/tables/`.
- The 18-test summary is recorded in `artifacts/test_summary.txt`.

## Conclusion and limitations

The shared offline path and the zero-noise Trilateration baseline are reproducible from the recorded code/configuration. The provisional anchor geometry, lack of a recorded full MATLAB numerical equivalence sweep, and lack of frozen dependency metadata limit the claim to a NEXUS synthetic smoke/comparison run.
