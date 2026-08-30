# E012 artifacts

The canonical generated bundle is kept in
`outputs/carla_uwb_reproduction/` so the documented command can reproduce it.
The files below are the evidence paths and SHA256 values from the 2026-08-29
run.

| File | SHA256 | Meaning |
|---|---|---|
| `carla_uwb_overhead.png` | `09cdda4ec266dcbe79cd3df3f2ffba3e0f9e2e243636412c855286c4f0a5b0d5` | CARLA RGB top-down projection with annotations |
| `carla_uwb_overhead.json` | `04e72dfd63a1bc42899536853053bf9338da4326e6cff70954195b50a00ba02e` | Visualization metadata and estimate rows |
| `observations.csv` | `05099de9ecae07246917e9f28153bc2603e9147777476154240436eebd03a321` | Algorithm-facing 3D range observations |
| `ground_truth.csv` | `55d0db55cf375d41f8d644a3049cce8d6fad2132983871b0496d6a0fe8927d00` | Independent CARLA actor transform |
| `estimates.csv` | `0e9de8609b787144af04611a1c1c424a75f74f8b6eace07e1f8af8aa01b9f7af` | Six route outputs |
| `metrics.json` | `ae5fc803f44b45fc2606134c0ee15c17c105b637fda3015bbdec9609c887bca2` | Metrics and run metadata |

The installed external map assets are not copied into Git. Their hashes are:

| Asset | SHA256 |
|---|---|
| `sandbox-v29/sandbox-v29.umap` | `92bdca80df4174344b6281bd3f6e58efd3beb429a7f48477af49ce3224a9fa7f` |
| `sandbox-v29/sandbox-v29.uexp` | `3e71946afaf364eb94bd26cea540acb374463dec82524dbad8b99b10174df9cc` |
| `sandbox-v29/OpenDrive/sandbox-v29.xodr` | `a2d0c3a33fad7012c08c7474e492a0e5cec5f0daa3ab1df7ff4aede40b699ec0` |

These outputs are CARLA simulation/test-sample evidence, not physical UWB
measurements or a surveyed map accuracy claim.
