import json
from pathlib import Path

import numpy as np


def centimeters_to_meters(values):
    array = np.asarray(values, dtype=float)
    if not np.all(np.isfinite(array)):
        raise ValueError("values must be finite")
    return array / 100.0


def meters_to_centimeters(values):
    array = np.asarray(values, dtype=float)
    if not np.all(np.isfinite(array)):
        raise ValueError("values must be finite")
    return array * 100.0


def ned_to_enu(values):
    array = np.asarray(values, dtype=float)
    if array.shape[-1] != 3 or not np.all(np.isfinite(array)):
        raise ValueError("NED values must be finite vectors with three components")
    return np.stack((array[..., 1], array[..., 0], -array[..., 2]), axis=-1)


def umeyama_alignment(source, target):
    source = np.asarray(source, dtype=float)
    target = np.asarray(target, dtype=float)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("source and target must have matching N x 3 shapes")
    if source.shape[0] < 3 or not np.all(np.isfinite(source)) or not np.all(np.isfinite(target)):
        raise ValueError("at least three finite point pairs are required")
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    covariance = (target - target_mean).T @ (source - source_mean) / source.shape[0]
    u, _, vh = np.linalg.svd(covariance)
    correction = np.eye(3)
    if np.linalg.det(u @ vh) < 0:
        correction[-1, -1] = -1.0
    rotation = u @ correction @ vh
    translation = target_mean - rotation @ source_mean
    return rotation, translation


def load_json_points(path):
    with open(path, "r", encoding="utf-8") as stream:
        document = json.load(stream)
    if "source" not in document or "target" not in document:
        raise ValueError("point file must contain source and target arrays")
    return np.asarray(document["source"], dtype=float), np.asarray(document["target"], dtype=float)


def load_calibration(path):
    """Load a registered YAML/JSON calibration document without inventing values."""
    path = Path(path)
    if path.suffix.lower() == ".json":
        with path.open("r", encoding="utf-8") as stream:
            document = json.load(stream)
    elif path.suffix.lower() in {".yaml", ".yml"}:
        import yaml
        with path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    else:
        raise ValueError("calibration file must be JSON or YAML")
    if not isinstance(document, dict):
        raise ValueError("calibration document must be an object")
    validate_calibration_version(document)
    return document


def validate_calibration_version(document, expected_major=1):
    version = document.get("calibration_version")
    if not isinstance(version, str) or not version:
        raise ValueError("calibration_version is required")
    try:
        major = int(version.split(".", 1)[0])
    except (TypeError, ValueError) as error:
        raise ValueError("calibration_version must use MAJOR.MINOR notation") from error
    if major != expected_major:
        raise ValueError(f"unsupported calibration major version: {major}")
    return version


def check_tf2_transforms(transforms):
    """Validate a tf2-style transform list; values remain caller-provided."""
    if not isinstance(transforms, list) or not transforms:
        raise ValueError("transforms must be a non-empty list")
    checked = []
    for item in transforms:
        if not isinstance(item, dict):
            raise ValueError("each transform must be an object")
        parent, child = item.get("parent"), item.get("child")
        translation, rotation = item.get("translation"), item.get("rotation_xyzw")
        if not isinstance(parent, str) or not parent or not isinstance(child, str) or not child:
            raise ValueError("transform parent and child frames are required")
        translation = np.asarray(translation, dtype=float)
        rotation = np.asarray(rotation, dtype=float)
        if translation.shape != (3,) or rotation.shape != (4,):
            raise ValueError("transform translation/rotation shapes must be 3 and 4")
        if not np.all(np.isfinite(translation)) or not np.all(np.isfinite(rotation)):
            raise ValueError("transform values must be finite")
        checked.append({"parent": parent, "child": child,
                        "translation": translation, "rotation_xyzw": rotation})
    return checked
