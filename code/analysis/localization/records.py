"""Per-solve output records for the three localization chains.

Column names follow the output contract in the markerless optimization design
(section 4.5).  This writes CSV/JSON only: no new ROS message type is
introduced, and chain differences live in the covariance and validity fields.
"""

from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .bundle_solver import BundleSolution


COLUMNS = (
    "frame_id", "obj_id", "class_name", "chain",
    "x_map_m", "y_map_m", "z_map_m", "qx", "qy", "qz", "qw",
    "sigma_x_m", "sigma_y_m", "sigma_z_m", "sigma_rot_deg",
    "n_views", "baseline_m", "depth_source",
    "validity", "degraded_reason", "runtime_ms",
)


@dataclass(frozen=True)
class SolveRecord:
    frame_id: str
    obj_id: int
    class_name: str
    chain: str
    x_map_m: float
    y_map_m: float
    z_map_m: float
    qx: float
    qy: float
    qz: float
    qw: float
    sigma_x_m: float | None
    sigma_y_m: float | None
    sigma_z_m: float | None
    sigma_rot_deg: float | None
    n_views: int
    baseline_m: float
    depth_source: str
    validity: str
    degraded_reason: str
    runtime_ms: float


def _sigmas(covariance: np.ndarray | None) -> tuple[float | None, float | None, float | None, float | None]:
    """Marginal one-sigma values from a [rotvec, translation] 6x6 block.

    A missing or non-positive diagonal is reported as ``None`` rather than as a
    fabricated zero, so a degraded solve cannot look confident downstream.
    """
    if covariance is None:
        return None, None, None, None
    diagonal = np.diag(np.asarray(covariance, dtype=float))
    if diagonal.shape != (6,) or not np.all(np.isfinite(diagonal)) or np.any(diagonal[:6] < 0.0):
        return None, None, None, None
    rotation_sigma = float(np.degrees(np.sqrt(np.mean(diagonal[:3]))))
    return float(np.sqrt(diagonal[3])), float(np.sqrt(diagonal[4])), float(np.sqrt(diagonal[5])), rotation_sigma


def _record(target_id: int, rotation: np.ndarray, translation: np.ndarray, covariance: np.ndarray | None, *,
            frame_id: str, chain: str, class_name: str, view_count: int, baseline_m: float,
            depth_source: str, valid: bool, degraded_reason: str, runtime_ms: float) -> SolveRecord:
    quaternion = Rotation.from_matrix(np.asarray(rotation, dtype=float)).as_quat()
    sigma_x, sigma_y, sigma_z, sigma_rotation = _sigmas(covariance)
    return SolveRecord(
        frame_id=str(frame_id), obj_id=int(target_id), class_name=str(class_name), chain=str(chain),
        x_map_m=float(translation[0]), y_map_m=float(translation[1]), z_map_m=float(translation[2]),
        qx=float(quaternion[0]), qy=float(quaternion[1]), qz=float(quaternion[2]), qw=float(quaternion[3]),
        sigma_x_m=sigma_x, sigma_y_m=sigma_y, sigma_z_m=sigma_z, sigma_rot_deg=sigma_rotation,
        n_views=int(view_count), baseline_m=float(baseline_m), depth_source=str(depth_source),
        validity="VALID" if valid else "INVALID", degraded_reason=str(degraded_reason), runtime_ms=float(runtime_ms),
    )


def solve_records(solution: BundleSolution, *, frame_id: str, chain: str,
                  class_names: dict[int, str] | None = None) -> list[SolveRecord]:
    """One record per solved target, in ascending object id."""
    names = class_names or {}
    return [_record(target_id, solution.target_poses[target_id].rotation, solution.target_poses[target_id].translation_m,
                    solution.target_covariances.get(target_id), frame_id=frame_id, chain=chain,
                    class_name=names.get(int(target_id), ""), view_count=solution.view_count,
                    baseline_m=solution.baseline_m, depth_source=solution.depth_source, valid=solution.valid,
                    degraded_reason=solution.degraded_reason, runtime_ms=solution.runtime_ms)
            for target_id in sorted(solution.target_poses)]


def result_records(result, *, frame_id: str, chain: str, class_names: dict[int, str] | None = None) -> list[SolveRecord]:
    """Records from an ``AlgorithmResult`` returned through the algorithm router."""
    names, metadata = class_names or {}, result.metadata
    return [_record(target_id, estimate[:3, :3], estimate[:3, 3], covariance, frame_id=frame_id, chain=chain,
                    class_name=names.get(int(target_id), ""), view_count=int(metadata.get("view_count", 0)),
                    baseline_m=float(metadata.get("baseline_m", 0.0)), depth_source=str(metadata.get("depth_source", "")),
                    valid=bool(result.valid), degraded_reason=str(metadata.get("degraded_reason", "")),
                    runtime_ms=float(metadata.get("runtime_ms", 0.0)))
            for target_id, estimate, covariance in zip(metadata.get("target_ids", []), result.estimate, result.covariance)]


def write_records(path: str | Path, records: list[SolveRecord]) -> Path:
    """Write the contract CSV; a sibling ``.json`` keeps the same rows."""
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    rows = [asdict(record) for record in records]
    with destination.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(COLUMNS))
        writer.writeheader()
        writer.writerows(rows)
    destination.with_suffix(".json").write_text(json.dumps(rows, indent=2) + "\n", encoding="utf-8")
    return destination
