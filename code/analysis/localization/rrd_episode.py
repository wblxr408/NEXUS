"""Adapter from a CARLA RRD capture episode into the localization bundle.

The RRD episode layout (``simulation/carla/README_rrd_uav_uwb_dataset.md``) is
not BOP: there are no instance masks, no depth and no target meshes, only a
monocular RGB stream, per-target image bounding boxes, synthetic UWB ranges and
CARLA truth.  Under those inputs the factor set degrades in a specific, bounded
way, and that is the whole point of moving depth off the network:

* F1 survives as a QuadricSLAM-style bbox-centre bearing, so multi-view depth
  intersection still works and the depth budget is unchanged.
* F3 survives from a declared support elevation and nominal target height
  (``config/rrd_target_priors_v01.yaml``), the same role the parametric catalog
  played -- a design prior, never per-frame truth.
* F6/F7 survive from the UWB ranges.
* **F2 and the dense F1 weights do not survive**: the contour factor needs a
  target mesh and a mask, and neither exists here.  Target *rotation* is
  therefore not observable from these inputs and must not be reported.

Coordinates: CARLA world is left-handed (x forward, y right, z up).  The NEXUS
``map`` frame is right-handed, so every location and rotation is converted by
flipping y.  Conversions are explicit and unit-tested because a silent handedness
error here is indistinguishable from a localization failure.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterator

import numpy as np
import yaml

from uwb.multilateration import solve_multilateration

from .factors import Factor, PlatformPoseFactor, SupportPlaneFactor
from .gating import QualityMetrics
from .observations import bbox_bearing_factor
from .bundle_solver import PoseState


# Left-handed CARLA world -> right-handed map: flip the y axis.
CARLA_TO_MAP = np.diag([1.0, -1.0, 1.0])
# CARLA actor axes (x forward, y right, z up) -> optical axes (x right, y down,
# z forward), expressed as optical axes in the actor basis.
ACTOR_TO_OPTICAL = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def carla_rotation_matrix(pitch_deg: float, yaw_deg: float, roll_deg: float) -> np.ndarray:
    """Rotation matrix matching ``carla.Rotation.get_matrix()``."""
    pitch, yaw, roll = (math.radians(value) for value in (pitch_deg, yaw_deg, roll_deg))
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    cr, sr = math.cos(roll), math.sin(roll)
    return np.array([
        [cp * cy, cy * sp * sr - sy * cr, -cy * sp * cr - sy * sr],
        [cp * sy, sy * sp * sr + cy * cr, -sy * sp * cr + cy * sr],
        [sp, -cp * sr, cp * cr],
    ])


def location_to_map(location: dict[str, float]) -> np.ndarray:
    return CARLA_TO_MAP @ np.array([float(location["x"]), float(location["y"]), float(location["z"])])


def optical_pose_to_map(transform: dict[str, Any]) -> PoseState:
    """CARLA camera transform -> ``T_map_optical`` with a proper rotation."""
    rotation = transform["rotation"]
    actor = carla_rotation_matrix(float(rotation["pitch"]), float(rotation["yaw"]), float(rotation["roll"]))
    return PoseState(CARLA_TO_MAP @ actor @ ACTOR_TO_OPTICAL, location_to_map(transform["location"]))


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


@dataclass(frozen=True)
class TargetPrior:
    """Declared, non-truth geometry for one RRD target."""

    target_id: str
    object_id: int
    support_z_m: float
    height_m: float
    sigma_support_m: float
    nominal_position_map_m: np.ndarray | None


def load_target_priors(path: str | Path) -> dict[str, TargetPrior]:
    document = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if document.get("source") != "design_prior_not_measured":
        raise ValueError("RRD target priors must declare source: design_prior_not_measured")
    priors = {}
    for entry in document["targets"]:
        nominal = entry.get("nominal_position_map_m")
        priors[str(entry["target_id"])] = TargetPrior(
            target_id=str(entry["target_id"]), object_id=int(entry["object_id"]),
            support_z_m=float(entry["support_z_m"]), height_m=float(entry["height_m"]),
            sigma_support_m=float(entry.get("sigma_support_m", document["default_sigma_support_m"])),
            nominal_position_map_m=None if nominal is None else np.asarray(nominal, dtype=float),
        )
    return priors


@dataclass(frozen=True)
class EpisodeFrame:
    frame: int
    camera_matrix: np.ndarray
    platform_pose: PoseState
    platform_source: str
    uwb_position_m: np.ndarray | None
    observations: list[dict[str, Any]]


class RrdEpisode:
    """Read-only view of one capture episode.

    ``platform_source='uwb'`` keeps the solver free of CARLA truth, which is what
    an online run can actually do.  ``'truth'`` is available only as a
    no-platform-error upper bound and is labelled as such in every record.
    """

    def __init__(self, root: str | Path, priors_path: str | Path, *, platform_source: str = "uwb"):
        if platform_source not in {"uwb", "truth"}:
            raise ValueError("platform_source must be 'uwb' or 'truth'")
        self.root = Path(root)
        self.platform_source = platform_source
        self.priors = load_target_priors(priors_path)
        calibration = json.loads((self.root / "calibration" / "camera.json").read_text(encoding="utf-8"))
        self.camera_matrix = np.asarray(calibration["camera_matrix"], dtype=float)
        self.image_size = (int(calibration["width"]), int(calibration["height"]))
        anchors = json.loads((self.root / "uwb" / "anchors.json").read_text(encoding="utf-8"))
        self.anchor_positions_map_m = np.asarray([location_to_map(item["location"]) for item in anchors["anchors"]], dtype=float)
        self.ranges = {int(row["frame"]): row for row in read_jsonl(self.root / "uwb" / "ranges.jsonl")}
        self.platform_attitudes: dict[int, np.ndarray] = {}
        attitude_path = self.root / "platform_attitude.csv"
        if platform_source == "uwb":
            if not attitude_path.is_file():
                raise ValueError("platform_source='uwb' requires platform_attitude.csv; refusing CARLA truth fallback")
            import csv
            with attitude_path.open(encoding="utf-8", newline="") as stream:
                for row in csv.DictReader(stream):
                    rotation = np.array(
                        [float(row[f"base_R_map_{r}{c}"]) for r in range(3) for c in range(3)],
                        dtype=float,
                    ).reshape(3, 3)
                    if not np.all(np.isfinite(rotation)) or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
                        raise ValueError(f"invalid platform attitude for frame {row.get('sequence')}")
                    self.platform_attitudes[int(row["sequence"])] = rotation
        self.observations: dict[int, list[dict[str, Any]]] = {}
        for row in read_jsonl(self.root / "vision_target_observations.jsonl"):
            self.observations.setdefault(int(row["frame"]), []).append(row)
        self.frame_ids = sorted(self.observations)
        if not self.frame_ids:
            raise ValueError(f"no vision target observations in {self.root}")

    def _uwb_position(self, frame: int) -> np.ndarray | None:
        row = self.ranges.get(frame)
        if row is None:
            return None
        usable = [anchor for anchor in row["anchors"] if anchor.get("valid") and not anchor.get("packet_lost")]
        if len(usable) < 4:
            return None
        anchors = np.asarray([location_to_map(anchor["anchor_location"]) for anchor in usable], dtype=float)
        ranges = np.asarray([float(anchor["raw_range_m"]) for anchor in usable], dtype=float)
        try:
            return solve_multilateration(anchors, ranges)
        except ValueError:
            return None

    def frames(self) -> Iterator[EpisodeFrame]:
        for frame in self.frame_ids:
            observations = self.observations[frame]
            # camera_to_map is CARLA truth and is only allowed in the explicit
            # upper-bound mode.
            truth_pose = optical_pose_to_map(observations[0]["camera_to_map"])
            uwb_position = self._uwb_position(frame)
            if self.platform_source == "truth":
                pose, source = truth_pose, "carla_truth_upper_bound"
            elif uwb_position is None:
                continue
            else:
                attitude = self.platform_attitudes.get(frame)
                if attitude is None:
                    continue
                # platform_attitude.csv stores R_base_map (map vectors into the
                # platform/camera frame); PoseState needs R_map_camera.
                pose, source = PoseState(attitude.T, uwb_position), "uwb_position_with_platform_attitude"
            yield EpisodeFrame(frame, self.camera_matrix, pose, source, uwb_position, observations)

    def frame_factors(self, episode_frame: EpisodeFrame, camera_id: int) -> tuple[list[Factor], QualityMetrics]:
        """F1 + F3 + F7 for one frame; F2/F6 need inputs absent from RRD."""
        factors: list[Factor] = []
        areas = []
        for observation in episode_frame.observations:
            prior = self.priors.get(str(observation["target_id"]))
            if prior is None or not observation.get("valid"):
                continue
            bbox = observation["bbox"].get("bbox_xyxy_clipped") or observation["bbox"].get("bbox_xyxy")
            if not bbox:
                continue
            x0, y0, x1, y1 = (float(value) for value in bbox)
            width, height = x1 - x0, y1 - y0
            if width <= 0.0 or height <= 0.0:
                continue
            areas.append(width * height)
            # A bbox centre is a coarse bearing, so its sigma is several pixels
            # rather than the sub-pixel figure a dense correspondence would get.
            factors.append(bbox_bearing_factor(target_id=prior.object_id, camera_id=camera_id,
                                               bbox_xywh_px=np.array([x0, y0, width, height]),
                                               camera_matrix=episode_frame.camera_matrix,
                                               confidence=1.0, pixel_sigma=4.0))
            factors.append(SupportPlaneFactor(prior.object_id, prior.height_m / 2.0, prior.support_z_m,
                                             base_offset_m=0.0, sigma_m=prior.sigma_support_m))
        if episode_frame.uwb_position_m is not None:
            factors.append(PlatformPoseFactor(camera_id, episode_frame.platform_pose.rotation,
                                              episode_frame.platform_pose.translation_m,
                                              sigma_rotation_rad=0.05, sigma_translation_m=0.10))
        quality = QualityMetrics(
            n_views_valid=len(areas), baseline_m=0.0, bbox_area_px=float(np.mean(areas)) if areas else 0.0,
            confidence=1.0 if areas else 0.0, pose_sigma_m=0.0, channel_disagreement_m=0.0,
            uwb_residual_m=0.0, platform_cov_trace_m2=0.0, blur_metric=1e3, exposure_metric=0.5,
            transform_available=episode_frame.uwb_position_m is not None or self.platform_source == "truth",
        )
        return factors, quality

    def target_truth_map_m(self) -> dict[int, np.ndarray]:
        """Evaluation-only map-frame target positions, averaged over the episode."""
        accumulated: dict[int, list[np.ndarray]] = {}
        for row in read_jsonl(self.root / "ground_truth" / "target_pose.jsonl"):
            prior = self.priors.get(str(row["target_id"]))
            if prior is None:
                continue
            accumulated.setdefault(prior.object_id, []).append(location_to_map(row["target_pose"]["location"]))
        return {key: np.mean(np.stack(value), axis=0) for key, value in accumulated.items()}


def initial_target_poses(priors: dict[str, TargetPrior], platform_pose: PoseState) -> dict[int, PoseState]:
    """Start each target on its support plane under the platform.

    The initial guess must not come from truth.  A nominal catalog position is
    used when the prior file declares one; otherwise the platform's horizontal
    position is the only unbiased starting point available online.
    """
    poses = {}
    for prior in priors.values():
        if prior.nominal_position_map_m is not None:
            translation = prior.nominal_position_map_m.copy()
        else:
            translation = np.array([platform_pose.translation_m[0], platform_pose.translation_m[1],
                                    prior.support_z_m + prior.height_m / 2.0])
        poses[prior.object_id] = PoseState(np.eye(3), translation)
    return poses
