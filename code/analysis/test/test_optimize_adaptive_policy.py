"""Contract test for replay-derived Pareto compute-policy profiles."""

import importlib.util
import json
import sys
from pathlib import Path


def _load_optimizer():
    path = Path(__file__).parents[1] / "vision" / "optimize_adaptive_policy.py"
    spec = importlib.util.spec_from_file_location("optimize_adaptive_policy", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_optimizer_writes_latency_feasible_profile(tmp_path, monkeypatch):
    records = []
    for index in range(24):
        hard = index % 2 == 0
        records.append({"features": {"identity_confidence": .1 if hard else .95,
                                     "inlier_ratio": .3 if hard else .95,
                                     "reprojection_error_px": 4. if hard else .2,
                                     "blur_metric": .3 if hard else .9,
                                     "vibration_quality": .4 if hard else .9,
                                     "occlusion_ratio": .5 if hard else .0,
                                     "outlier_probability": .8 if hard else .0},
                        "requires_high_accuracy": hard, "lite_latency_ms": 20., "high_latency_ms": 70.})
    source, output = tmp_path / "records.jsonl", tmp_path / "profile.json"
    source.write_text("\n".join(json.dumps(item) for item in records) + "\n", encoding="utf-8")
    module = _load_optimizer()
    monkeypatch.setattr(sys, "argv", ["optimize_adaptive_policy.py", "--records", str(source), "--output", str(output),
                                       "--p95-latency-ms", "80"])
    module.main()
    profile = json.loads(output.read_text(encoding="utf-8"))
    assert profile["fit"]["records"] == 24
    assert profile["fit"]["empirical_p95_latency_ms"] <= 80.
    assert set(profile["risk_weights"]) == set(module.NAMES)
