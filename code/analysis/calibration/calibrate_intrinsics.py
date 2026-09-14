"""Calibrate a fixed camera mode from measured checkerboard views, with holdout."""

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def solve_views(objects, images, size):
    if len(images) < 20:
        raise ValueError("at least 20 diverse detected board views are required")
    holdout = list(range(4, len(images), 5))
    train = [i for i in range(len(images)) if i not in holdout]
    rms, matrix, distortion, rotations, _ = cv2.calibrateCamera(
        [objects[i] for i in train], [images[i] for i in train], size, None, None)
    normals = np.array([cv2.Rodrigues(rotation)[0][:, 2] for rotation in rotations])
    angular_spread = float(np.degrees(np.arccos(np.clip((normals @ normals.T).min(), -1, 1))))
    if angular_spread < 15:
        raise ValueError("insufficient board tilt diversity (less than 15 degrees)")
    errors = []
    for i in holdout:
        ok, rotation, translation = cv2.solvePnP(objects[i], images[i], matrix, distortion)
        if not ok:
            raise ValueError("held-out board pose failed")
        projected, _ = cv2.projectPoints(objects[i], rotation, translation, matrix, distortion)
        errors.append(float(np.sqrt(np.mean(np.sum((projected - images[i]) ** 2, axis=2)))))
    if not np.isfinite(matrix).all() or not np.isfinite(distortion).all():
        raise ValueError("non-finite calibration")
    if max(errors) > 1.0 or rms > 1.0:
        raise ValueError(f"calibration reprojection gate failed: train={rms}, holdout={errors}")
    # Retain the training fit: holdout views never enter its intrinsic optimization.
    return matrix, distortion, dict(train_rms_px=rms, holdout_rms_px=errors,
                                    board_normal_spread_deg=angular_spread,
                                    training_indices=train, holdout_indices=holdout)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--exclude", action="append", default=[], metavar="VIEW",
                        help="explicitly exclude a named verified-bad view, e.g. view_019")
    parser.add_argument("--valid-roi", nargs=4, type=float,
                        metavar=("X0", "Y0", "X1", "Y1"),
                        help="normalized measured-valid image region; default requires full image coverage")
    args = parser.parse_args()
    images, objects, sources = [], [], []
    excluded = set(args.exclude)
    excluded_sources = []
    mode = None
    for path in sorted(args.input.glob("view_*.jpg")):
        if path.stem in excluded:
            excluded_sources.append(path.name)
            continue
        metadata = json.loads(path.with_suffix(".json").read_text())
        current_mode = (metadata["width"], metadata["height"], metadata["rotation_deg"],
                        tuple(metadata["board_inner_corners"]), metadata["square_size_m"])
        if mode is not None and current_mode != mode:
            raise ValueError("mixed camera modes or checkerboard specifications")
        mode = current_mode
        if hashlib.sha256(path.read_bytes()).hexdigest() != metadata["image_sha256"]:
            raise ValueError("source image checksum changed")
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None or gray.shape != (mode[1], mode[0]):
            raise ValueError("image dimensions differ from capture metadata")
        found, corners = cv2.findChessboardCornersSB(gray, mode[3], cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            raise ValueError(f"checkerboard not detected in {path.name}")
        pattern = np.zeros((mode[3][0] * mode[3][1], 3), np.float32)
        pattern[:, :2] = np.mgrid[:mode[3][0], :mode[3][1]].T.reshape(-1, 2) * mode[4]
        images.append(corners)
        objects.append(pattern)
        sources.append(dict(file=path.name, sha256=metadata["image_sha256"]))
    if mode is None:
        raise ValueError("no captured calibration views")
    roi = np.asarray(args.valid_roi if args.valid_roi is not None else [0.0, 0.0, 1.0, 1.0],
                     dtype=float)
    if (not np.isfinite(roi).all() or np.any(roi < 0.0) or np.any(roi > 1.0)
            or roi[0] >= roi[2] or roi[1] >= roi[3]):
        raise ValueError("valid ROI must be finite normalized X0 Y0 X1 Y1 with increasing bounds")
    matrix, distortion, validation = solve_views(objects, images, mode[:2])
    points = np.concatenate(images).reshape(-1, 2) / np.array(mode[:2])
    # Require measured coverage across the enabled output region, including a 5% margin.
    if (np.any(points.min(axis=0) > roi[:2] + .05)
            or np.any(points.max(axis=0) < roi[2:] - .05)):
        raise ValueError("insufficient coverage within valid ROI: move board across its four edges")
    result = dict(schema_version=1, calibration_status="measured", camera_model="IMX219",
                  width=mode[0], height=mode[1], rotation_deg=mode[2],
                  board_inner_corners=list(mode[3]), square_size_m=mode[4],
                  distortion_model="plumb_bob", camera_matrix=matrix.tolist(),
                  distortion_coefficients=distortion.reshape(-1).tolist(),
                  valid_roi_normalized=roi.tolist(), sources=sources,
                  excluded_sources=excluded_sources, validation=validation,
                  scope="intrinsics_only_no_camera_imu_extrinsics_or_temporal_calibration")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps(validation, indent=2))


if __name__ == "__main__":
    main()
