# Processed data registry

Generated simulation datasets are intentionally ignored by Git. Each run
records the exact output directory, generator/config commit, random seed and
SHA256 values in `experiments/runs/`.

Current generated dataset:

- `nexus_sandbox_gdr_net_v01/`: five deterministic platform camera poses,
  ten fixed central targets, RGB/instance masks, BOP-style camera/GT files,
  simplified PLY models and UWB ranges.
- Generation command: `python3 simulation/generate_gdr_net_dataset.py
  --out data/processed/nexus_sandbox_gdr_net_v01`
- This is `metric_simulation` geometry-only data, not a surveyed CAD model or
  real-sandbox accuracy evidence.
