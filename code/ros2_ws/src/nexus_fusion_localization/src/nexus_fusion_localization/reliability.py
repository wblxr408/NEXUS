"""40 -> 64 -> 32 -> 4 observation-quality MLP with NumPy inference/training.

Twenty normalized measurements plus twenty availability flags allow individual
sensors to be absent without pretending their quality was measured as zero.
Labels must come from a separately documented error/quality annotation process;
this module does not create ground truth from predictions or nearby frames.
"""

import json
from pathlib import Path

import numpy as np


FEATURE_NAMES = (
    "tag_area_px", "corner_quality", "reprojection_error_px", "inlier_ratio",
    "visible_points", "tilt_rad", "tracked_points", "match_distance",
    "forward_backward_error_px", "feature_coverage", "uwb_innovation_m",
    "anchor_count", "uwb_jump_m", "gdop", "imu_motion_residual",
    "blur_metric", "occlusion_ratio", "dt_s", "network_latency_s", "speed_mps",
)
# These are valid observation diagnostics used by the target-fusion and
# adaptive-scheduling paths.  They deliberately do not enter the fixed
# 20-dimensional learned-reliability model, so an existing checkpoint keeps
# its declared feature schema and dimensionality.
SUPPLEMENTAL_FEATURE_NAMES = (
    "identity_confidence", "outlier_probability", "vibration_quality",
    "exposure_rotation_sigma_rad",
)
OUTPUT_NAMES = ("vision_reliability", "uwb_reliability", "imu_reliability", "outlier_probability")


def feature_row(values):
    unknown = set(values) - set(FEATURE_NAMES) - set(SUPPLEMENTAL_FEATURE_NAMES)
    if unknown:
        raise ValueError(f"unknown quality features: {sorted(unknown)}")
    result = np.array([np.nan if values.get(name) is None else float(values[name])
                       for name in FEATURE_NAMES])
    if np.any(np.isinf(result)):
        raise ValueError("quality features cannot be infinite")
    return result


def covariance_scales(reliability, maximum_scale=100.0):
    """Independent vision/UWB/IMU inflations; never reduce baseline covariance."""
    values = np.asarray(reliability, dtype=float)
    if (values.shape != (4,) or not np.all(np.isfinite(values))
            or np.any(values < 0) or np.any(values > 1)
            or not np.isfinite(maximum_scale) or maximum_scale < 1):
        raise ValueError("reliability requires four probabilities and a finite scale >= 1")
    trust = values[:3] * (1.0 - values[3])
    return 1.0 + (maximum_scale - 1.0) * (1.0 - trust) ** 2


def split_groups(groups, seed=0):
    """Split complete collection/flight groups 60/20/20; never individual frames."""
    groups = np.asarray(groups, dtype=str)
    if groups.ndim != 1 or any(not group.strip() for group in groups):
        raise ValueError("each row requires a nonempty collection group")
    unique = np.unique(groups)
    if len(unique) < 3:
        raise ValueError("at least three independent collection groups are required")
    unique = np.random.default_rng(seed).permutation(unique)
    n_test = max(1, int(len(unique) * .2))
    n_validation = max(1, int(len(unique) * .2))
    assignments = {"test": unique[:n_test], "validation": unique[n_test:n_test + n_validation],
                   "train": unique[n_test + n_validation:]}
    return {name: np.flatnonzero(np.isin(groups, assigned))
            for name, assigned in assignments.items()}


def _sigmoid(values):
    return 1.0 / (1.0 + np.exp(-np.clip(values, -60, 60)))


def _scores(predicted, labels):
    bounded = np.clip(predicted, 1e-8, 1 - 1e-8)
    return {"binary_cross_entropy": float(-np.mean(labels * np.log(bounded)
                                                   + (1 - labels) * np.log(1 - bounded))),
            "brier_score": float(np.mean((predicted - labels) ** 2)),
            "per_output_brier": dict(zip(OUTPUT_NAMES, np.mean((predicted - labels) ** 2, axis=0).tolist()))}


class ReliabilityMLP:
    def __init__(self, seed=0):
        rng = np.random.default_rng(seed)
        self.weights = [rng.normal(0, np.sqrt(2 / a), (a, b)) for a, b in ((40, 64), (64, 32), (32, 4))]
        self.biases = [np.zeros(size) for size in (64, 32, 4)]
        self.mean = np.zeros(20)
        self.scale = np.ones(20)
        self.metadata = {}
        self.trained = False

    def _normalize(self, features):
        values = np.asarray(features, dtype=float)
        if values.ndim != 2 or values.shape[1] != 20 or np.any(np.isinf(values)):
            raise ValueError("quality data must have shape N x 20 with finite or missing values")
        available = np.isfinite(values)
        normalized = np.where(available, (values - self.mean) / self.scale, 0.0)
        return np.column_stack((normalized, available.astype(float)))

    def _forward(self, normalized):
        first = np.maximum(0, normalized @ self.weights[0] + self.biases[0])
        second = np.maximum(0, first @ self.weights[1] + self.biases[1])
        output = _sigmoid(second @ self.weights[2] + self.biases[2])
        return first, second, output

    def predict(self, features):
        if not self.trained:
            raise RuntimeError("reliability model has no trained checkpoint")
        if isinstance(features, dict):
            return self._forward(self._normalize(feature_row(features)[None]))[2][0]
        return self._forward(self._normalize(features))[2]

    def fit(self, features, labels, groups, *, epochs=100, batch_size=64,
            learning_rate=.001, seed=0, label_provenance):
        values, labels = np.asarray(features, dtype=float), np.asarray(labels, dtype=float)
        self._normalize(values)
        if (labels.shape != (len(values), 4) or not np.all(np.isfinite(labels))
                or np.any(labels < 0) or np.any(labels > 1) or len(groups) != len(values)):
            raise ValueError("labels must be N x 4 probabilities, with one group per row")
        if (not label_provenance or epochs < 1 or batch_size < 1
                or not np.isfinite(learning_rate) or learning_rate <= 0):
            raise ValueError("training requires label provenance and positive settings")
        split = split_groups(groups, seed)
        train = values[split["train"]]
        available = np.isfinite(train)
        count = np.maximum(1, available.sum(axis=0))
        self.mean = np.where(available, train, 0).sum(axis=0) / count
        variance = np.where(available, (train - self.mean) ** 2, 0).sum(axis=0) / count
        self.scale = np.where(variance > 1e-12, np.sqrt(variance), 1.0)
        normalized = self._normalize(values)
        parameters = self.weights + self.biases
        momentum = [np.zeros_like(parameter) for parameter in parameters]
        second_moment = [np.zeros_like(parameter) for parameter in parameters]
        rng, step = np.random.default_rng(seed), 0
        best_validation, best_parameters, best_epoch = float("inf"), None, 0
        initial_validation = _scores(self._forward(normalized[split["validation"]])[2], labels[split["validation"]])
        for epoch in range(int(epochs)):
            indices = rng.permutation(split["train"])
            for start in range(0, len(indices), batch_size):
                batch = indices[start:start + batch_size]
                x, y = normalized[batch], labels[batch]
                first, second, output = self._forward(x)
                delta3 = (output - y) / y.size
                delta2 = (delta3 @ self.weights[2].T) * (second > 0)
                delta1 = (delta2 @ self.weights[1].T) * (first > 0)
                gradients = [x.T @ delta1, first.T @ delta2, second.T @ delta3,
                             delta1.sum(axis=0), delta2.sum(axis=0), delta3.sum(axis=0)]
                step += 1
                for i, (parameter, gradient) in enumerate(zip(parameters, gradients)):
                    momentum[i] = .9 * momentum[i] + .1 * gradient
                    second_moment[i] = .999 * second_moment[i] + .001 * gradient ** 2
                    parameter -= learning_rate * (momentum[i] / (1 - .9 ** step)) / (
                        np.sqrt(second_moment[i] / (1 - .999 ** step)) + 1e-8)
            validation = _scores(self._forward(normalized[split["validation"]])[2], labels[split["validation"]])
            if validation["binary_cross_entropy"] < best_validation:
                best_validation = validation["binary_cross_entropy"]
                best_parameters = [parameter.copy() for parameter in parameters]
                best_epoch = epoch + 1
        for parameter, best in zip(parameters, best_parameters):
            parameter[:] = best
        self.trained = True
        group_array = np.asarray(groups, dtype=str)
        self.metadata = {"format_version": 1, "feature_names": FEATURE_NAMES,
                         "output_names": OUTPUT_NAMES, "architecture": [40, 64, 32, 4],
                         "seed": seed, "epochs": epochs, "selected_epoch": best_epoch,
                         "label_provenance": str(label_provenance),
                         "split_groups": {name: np.unique(group_array[indices]).tolist()
                                          for name, indices in split.items()},
                         "split_counts": {name: len(indices) for name, indices in split.items()},
                         "initial_validation": initial_validation,
                         "metrics": {name: _scores(self.predict(values[indices]), labels[indices])
                                     for name, indices in split.items()}}
        return self.metadata

    def save(self, path):
        if not self.trained:
            raise RuntimeError("refusing to save untrained reliability model")
        path = Path(path)
        # Exclusive creation prevents accidental overwrite of experiment models.
        with path.open("xb") as stream:
            np.savez_compressed(stream, mean=self.mean, scale=self.scale,
                                metadata=json.dumps(self.metadata),
                                **{f"w{i}": value for i, value in enumerate(self.weights)},
                                **{f"b{i}": value for i, value in enumerate(self.biases)})

    @classmethod
    def load(cls, path):
        model = cls()
        with np.load(path, allow_pickle=False) as data:
            metadata = json.loads(str(data["metadata"]))
            if (metadata.get("format_version") != 1 or metadata.get("architecture") != [40, 64, 32, 4]
                    or tuple(metadata.get("feature_names", ())) != FEATURE_NAMES
                    or tuple(metadata.get("output_names", ())) != OUTPUT_NAMES
                    or not metadata.get("label_provenance")):
                raise ValueError("incompatible reliability checkpoint schema")
            model.mean, model.scale = data["mean"].copy(), data["scale"].copy()
            for name, values in (("w", model.weights), ("b", model.biases)):
                for i, expected in enumerate(values):
                    value = data[f"{name}{i}"]
                    if value.shape != expected.shape or not np.all(np.isfinite(value)):
                        raise ValueError("invalid reliability checkpoint parameters")
                    values[i] = value.copy()
            if (model.mean.shape != (20,) or model.scale.shape != (20,)
                    or not np.all(np.isfinite(model.mean)) or not np.all(np.isfinite(model.scale))
                    or np.any(model.scale <= 0)):
                raise ValueError("invalid checkpoint normalization")
            model.metadata, model.trained = metadata, True
        return model
