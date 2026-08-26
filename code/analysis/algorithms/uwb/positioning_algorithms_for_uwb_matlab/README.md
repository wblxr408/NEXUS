# positioning-algorithms-for-uwb-matlab adapter

Source: <https://github.com/cliansang/positioning-algorithms-for-uwb-matlab>

This directory contains the Python adapters for the five positioning families used by NEXUS:

- `trilateration.py` -> `uwb.matlab.trilateration`
- `multilateration.py` -> `uwb.matlab.multilateration`
- `taylor_series.py` -> `uwb.matlab.taylor`
- `kalman_filters.py` -> `uwb.matlab.ekf` and `uwb.matlab.ukf`

The reference repository is kept as the initialized submodule at `code/analysis/uwb/third_party/positioning_algorithms_for_uwb_matlab/`. The adapter code does not modify that submodule. Register concrete runners through `register_default_algorithms()` and run them through `AlgorithmRouter`; do not pass Ground Truth to an adapter.
