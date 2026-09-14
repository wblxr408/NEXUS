"""Fit a reproducible Pareto compute-policy profile from labelled replay logs.

Each JSONL record must contain ``features`` with the scheduler-quality fields,
``requires_high_accuracy`` (ground-truth failure label), ``lite_latency_ms``
and ``high_latency_ms``.  The selected profile minimizes a weighted
false-negative/extra-compute loss subject to an empirical P95 latency budget.
It is therefore optimal only for the supplied records and declared objective,
never a claim of universal optimality.
"""

import argparse
import json
from pathlib import Path

import numpy as np


NAMES = ("identity", "inliers", "reprojection", "sharpness", "vibration", "occlusion", "visibility", "outlier")


def risk_features(record):
    values = record["features"]
    identity = float(values.get("identity_confidence", 0.))
    inliers = float(values.get("inlier_ratio", 0.))
    reprojection = float(values.get("reprojection_error_px", 4.))
    sharpness = float(values.get("blur_metric", 0.))
    vibration = float(values.get("vibration_quality", 0.))
    occlusion = float(values.get("occlusion_ratio", 0.))
    visible = float(values.get("visibility", 1. - occlusion))
    outlier = float(values.get("outlier_probability", 0.))
    raw = np.array([1 - identity, 1 - inliers, min(1., reprojection / 4.), 1 - sharpness,
                    1 - vibration, occlusion, 1 - visible, outlier], dtype=float)
    if not np.all(np.isfinite(raw)) or np.any(raw < 0):
        raise ValueError("record contains invalid scheduler feature")
    return np.clip(raw, 0., 1.)


def fit_nonnegative_weights(features, labels, iterations=2000, learning_rate=.05):
    """Projected logistic fit, retaining interpretable nonnegative risks."""
    weights = np.full(features.shape[1], 1. / features.shape[1])
    for _ in range(iterations):
        score = features @ weights
        probability = 1. / (1. + np.exp(np.clip(-8. * (score - .5), -60, 60)))
        gradient = features.T @ (probability - labels) / len(features)
        weights = np.maximum(0., weights - learning_rate * gradient)
        weights /= max(1e-12, weights.sum())
    return weights


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--records", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--p95-latency-ms", type=float, required=True)
    parser.add_argument("--false-negative-cost", type=float, default=10.)
    parser.add_argument("--false-positive-cost", type=float, default=1.)
    args = parser.parse_args()
    if (not args.records.is_file() or args.output.exists() or not np.isfinite(args.p95_latency_ms) or args.p95_latency_ms <= 0
            or not np.isfinite(args.false_negative_cost) or args.false_negative_cost <= 0
            or not np.isfinite(args.false_positive_cost) or args.false_positive_cost <= 0):
        raise ValueError("optimization arguments are invalid")
    records = [json.loads(line) for line in args.records.read_text(encoding="utf-8").splitlines() if line.strip()]
    if len(records) < 20:
        raise ValueError("at least 20 labelled replay records are required")
    feature_matrix = np.stack([risk_features(record) for record in records])
    labels = np.asarray([record["requires_high_accuracy"] for record in records], dtype=float)
    lite = np.asarray([record["lite_latency_ms"] for record in records], dtype=float)
    high = np.asarray([record["high_latency_ms"] for record in records], dtype=float)
    if not np.all(np.isin(labels, [0, 1])) or not all(np.all(np.isfinite(value)) and np.all(value >= 0) for value in (lite, high)):
        raise ValueError("records require binary labels and finite nonnegative latencies")
    weights = fit_nonnegative_weights(feature_matrix, labels)
    scores = feature_matrix @ weights
    candidates = []
    for threshold in np.unique(np.quantile(scores, np.linspace(.02, .98, 97))):
        selected = scores >= threshold
        p95 = float(np.percentile(np.where(selected, high, lite), 95))
        false_negative = np.logical_and(~selected, labels == 1).mean()
        false_positive = np.logical_and(selected, labels == 0).mean()
        utility = args.false_negative_cost * false_negative + args.false_positive_cost * false_positive
        if p95 <= args.p95_latency_ms:
            candidates.append((utility, p95, float(threshold)))
    if not candidates:
        raise ValueError("no policy satisfies the requested P95 latency budget")
    utility, p95, threshold = min(candidates)
    document = {"schema_version": 1, "objective": {"p95_latency_ms": args.p95_latency_ms,
                "false_negative_cost": args.false_negative_cost, "false_positive_cost": args.false_positive_cost},
                "risk_weights": dict(zip(NAMES, weights.tolist())), "enter_threshold": threshold,
                "exit_threshold": max(0., threshold - .15),
                "fast_latency_ms": min(args.p95_latency_ms, 120.), "late_latency_ms": args.p95_latency_ms,
                "fit": {"records": len(records), "empirical_p95_latency_ms": p95, "objective_loss": utility}}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(document, ensure_ascii=False))


if __name__ == "__main__":
    main()
