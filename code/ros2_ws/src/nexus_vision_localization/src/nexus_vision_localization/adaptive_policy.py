"""Quality-driven edge compute and safe active-observation recommendations.

The policy is deliberately independent of ROS and flight control.  It turns
measured visual/identity/IMU quality into an explicit compute budget, and
turns the current platform/target geometry into a *recommendation*.  A flight
controller may display or approve that recommendation; this package never
publishes a vehicle command.
"""

from dataclasses import dataclass
import math

import numpy as np


def _vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite {size}-vector")
    return result


def blur_score(gray_image):
    """Return normalized Laplacian-variance sharpness in [0, 1]."""
    image = np.asarray(gray_image)
    if image.ndim != 2 or image.size < 64 or not np.all(np.isfinite(image)):
        raise ValueError("blur metric requires a finite grayscale image")
    laplacian = (-4 * image.astype(float)
                 + np.roll(image, 1, 0) + np.roll(image, -1, 0)
                 + np.roll(image, 1, 1) + np.roll(image, -1, 1))
    variance = float(np.var(laplacian[1:-1, 1:-1]))
    # Saturates rather than allowing a scene's texture alone to dominate.
    return float(variance / (variance + 100.0))


def gyro_vibration(angular_velocity_rad_s, sample_intervals_s):
    """Estimate high-frequency angular disturbance without assuming a bias model."""
    omega = np.asarray(angular_velocity_rad_s, dtype=float)
    intervals = np.asarray(sample_intervals_s, dtype=float)
    if omega.ndim != 2 or omega.shape[1] != 3 or len(omega) < 2 or intervals.shape != (len(omega) - 1,):
        raise ValueError("gyro vibration requires Nx3 samples and N-1 intervals")
    if not np.all(np.isfinite(omega)) or not np.all(np.isfinite(intervals)) or np.any(intervals <= 0):
        raise ValueError("gyro vibration inputs must be finite and positive")
    mean = np.average(omega, axis=0, weights=np.r_[intervals, intervals[-1]])
    residual = omega - mean
    rms = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
    angle_rms = float(rms * np.sqrt(np.mean(intervals ** 2)))
    return {"gyro_vibration_rad_s": rms, "image_rotation_sigma_rad": angle_rms,
            "vibration_quality": float(math.exp(-4.0 * angle_rms))}


def rotational_homography(camera_matrix, rotation_previous_current):
    """Image derotation H=K R K^-1 for a calibrated global-shutter image."""
    camera = np.asarray(camera_matrix, dtype=float)
    rotation = np.asarray(rotation_previous_current, dtype=float)
    if camera.shape != (3, 3) or rotation.shape != (3, 3) or not np.all(np.isfinite(camera)) or not np.all(np.isfinite(rotation)):
        raise ValueError("derotation requires finite 3x3 camera matrix and rotation")
    if abs(np.linalg.det(camera)) < 1e-9 or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5) or np.linalg.det(rotation) <= 0:
        raise ValueError("camera matrix or rotation is invalid")
    result = camera @ rotation @ np.linalg.inv(camera)
    return result / result[2, 2]


def rolling_shutter_homographies(camera_matrix, angular_velocity_camera_rad_s, image_rows,
                                 readout_s, exposure_s=0.):
    """Return row-to-centre image maps for IMU-driven rolling-shutter EIS.

    Each matrix maps a virtual centre-exposure pixel to the sampled raw row.
    Exposure integration cannot recreate information lost to motion blur, so
    the accompanying uncertainty is retained by the caller for covariance
    inflation rather than pretending rectification makes the frame sharp.
    """
    camera = np.asarray(camera_matrix, dtype=float)
    omega = _vector(angular_velocity_camera_rad_s, 3, "camera angular velocity")
    if not isinstance(image_rows, int) or image_rows < 2 or not np.isfinite(readout_s) or not np.isfinite(exposure_s):
        raise ValueError("rolling shutter rows/readout/exposure are invalid")
    if readout_s < 0 or exposure_s < 0:
        raise ValueError("rolling shutter times must be nonnegative")
    # Mid-exposure of each row relative to the frame midpoint.
    times = np.linspace(-.5 * readout_s, .5 * readout_s, image_rows)
    magnitude = float(np.linalg.norm(omega))
    maps = []
    for time_s in times:
        vector = -omega * time_s  # passive camera-coordinate rotation
        angle = float(np.linalg.norm(vector))
        if angle < 1e-12:
            rotation = np.eye(3)
        else:
            axis = vector / angle
            skew = np.array([[0., -axis[2], axis[1]], [axis[2], 0., -axis[0]], [-axis[1], axis[0], 0.]])
            rotation = np.eye(3) + math.sin(angle) * skew + (1. - math.cos(angle)) * (skew @ skew)
        maps.append(rotational_homography(camera, rotation))
    # Uniform exposure is an unobserved interval around each row midpoint.
    # This is an angular one-sigma proxy used only to increase uncertainty.
    exposure_sigma_rad = magnitude * math.sqrt(readout_s ** 2 + exposure_s ** 2) / math.sqrt(12.)
    return np.stack(maps), float(exposure_sigma_rad)


@dataclass(frozen=True)
class ComputePlan:
    detector_period_s: float
    maximum_keypoints: int
    maximum_rois: int
    transformer_tokens: int
    transformer_history: int
    image_scale: float
    high_accuracy: bool
    robust_model: bool
    reason: str

    def as_dict(self):
        return {"detector_period_s": self.detector_period_s, "maximum_keypoints": self.maximum_keypoints,
                "maximum_rois": self.maximum_rois, "transformer_tokens": self.transformer_tokens,
                "transformer_history": self.transformer_history, "image_scale": self.image_scale,
                "high_accuracy": self.high_accuracy, "robust_model": self.robust_model, "reason": self.reason}


class AdaptiveComputeScheduler:
    """Hysteretic policy: spend compute only when identity/geometry is weak."""
    _RISK_NAMES = ("identity", "inliers", "reprojection", "sharpness", "vibration", "occlusion", "visibility", "outlier")

    def __init__(self, *, minimum_detector_period_s=.5, maximum_detector_period_s=2., base_keypoints=256, profile=None):
        if not 0 < minimum_detector_period_s <= maximum_detector_period_s or base_keypoints < 64:
            raise ValueError("invalid adaptive compute limits")
        self.minimum_detector_period_s = float(minimum_detector_period_s)
        self.maximum_detector_period_s = float(maximum_detector_period_s)
        self.base_keypoints = int(base_keypoints)
        profile = {} if profile is None else dict(profile)
        weights = profile.get("risk_weights", {"identity": .20, "inliers": .17, "reprojection": .14,
                                                "sharpness": .10, "vibration": .08, "occlusion": .06,
                                                "visibility": .12, "outlier": .15})
        if set(weights) != set(self._RISK_NAMES) or any(not np.isfinite(value) or value < 0 for value in weights.values()):
            raise ValueError("adaptive profile requires finite nonnegative risk weights")
        total = sum(float(value) for value in weights.values())
        if total <= 0:
            raise ValueError("adaptive profile risk weights must have positive sum")
        # Keep the deployed default profile numerically stable.  The offline
        # optimizer emits unit-sum weights; hand-authored profiles may retain
        # an existing calibrated scale together with matching thresholds.
        self.risk_weights = {name: float(weights[name]) for name in self._RISK_NAMES}
        self.enter_threshold = float(profile.get("enter_threshold", .55))
        self.exit_threshold = float(profile.get("exit_threshold", .35))
        self.fast_latency_ms = float(profile.get("fast_latency_ms", 120.))
        self.late_latency_ms = float(profile.get("late_latency_ms", 160.))
        if (not all(np.isfinite(value) and value >= 0 for value in
                    (self.enter_threshold, self.exit_threshold, self.fast_latency_ms, self.late_latency_ms))
                or not 0 <= self.exit_threshold <= self.enter_threshold <= 1 or self.fast_latency_ms > self.late_latency_ms):
            raise ValueError("adaptive profile thresholds are invalid")
        self._high_accuracy = False

    def decide(self, *, identity_confidence=None, inlier_ratio=None, reprojection_error_px=None,
               blur_metric=None, vibration_quality=None, occlusion_ratio=None, visibility=None,
               outlier_probability=None, latency_ms=None, identity_ambiguous=False):
        values = (identity_confidence, inlier_ratio, reprojection_error_px, blur_metric, vibration_quality,
                  occlusion_ratio, visibility, outlier_probability, latency_ms)
        if any(value is not None and (not np.isfinite(value) or value < 0) for value in values):
            raise ValueError("quality values must be finite and nonnegative when available")
        identity = .0 if identity_confidence is None else min(1., float(identity_confidence))
        inliers = .0 if inlier_ratio is None else min(1., float(inlier_ratio))
        reprojection = 4. if reprojection_error_px is None else min(8., float(reprojection_error_px))
        sharpness = .0 if blur_metric is None else min(1., float(blur_metric))
        vibration = .0 if vibration_quality is None else min(1., float(vibration_quality))
        occlusion = 0. if occlusion_ratio is None else min(1., float(occlusion_ratio))
        visible = 1. - occlusion if visibility is None else min(1., float(visibility))
        outlier = 0. if outlier_probability is None else min(1., float(outlier_probability))
        latency = 0. if latency_ms is None else float(latency_ms)
        losses = {"identity": 1 - identity, "inliers": 1 - inliers, "reprojection": min(1., reprojection / 4.),
                  "sharpness": 1 - sharpness, "vibration": 1 - vibration, "occlusion": occlusion,
                  "visibility": 1. - visible, "outlier": outlier}
        risk = sum(self.risk_weights[name] * losses[name] for name in self._RISK_NAMES)
        if identity_ambiguous:
            risk = 1.
        # Avoid oscillating each frame around a single threshold.
        if self._high_accuracy:
            self._high_accuracy = risk > self.exit_threshold and latency < self.late_latency_ms
        else:
            self._high_accuracy = risk > self.enter_threshold and latency < self.fast_latency_ms
        if latency >= self.late_latency_ms:
            # Consumers keep a minimum attention budget so that a late plan
            # remains valid and can actually replace a more expensive plan.
            return ComputePlan(self.maximum_detector_period_s, self.base_keypoints, 1, 4, 1, .5, False, False,
                               "latency_budget_exceeded")
        if self._high_accuracy:
            # A separately trained difficult-scene detector is only selected
            # once the risk is severe; ordinary uncertainty retains the full
            # detector as a higher-accuracy fallback.
            return ComputePlan(self.minimum_detector_period_s, self.base_keypoints * 2, 4, 64, 8, 1., True, risk >= .75,
                               "identity_or_geometry_uncertain")
        return ComputePlan(self.maximum_detector_period_s, self.base_keypoints, 2, 24, 4, .5, False, False,
                           "stable_tracking")


@dataclass(frozen=True)
class ObservationRecommendation:
    position_map_m: np.ndarray
    yaw_rad: float
    orbit_radius_m: float
    score: float
    reason: str
    trajectory_map_m: np.ndarray
    travel_time_s: float

    def as_dict(self):
        return {"position_map_m": self.position_map_m.tolist(), "yaw_rad": self.yaw_rad,
                "orbit_radius_m": self.orbit_radius_m, "score": self.score, "reason": self.reason,
                "trajectory_map_m": self.trajectory_map_m.tolist(), "travel_time_s": self.travel_time_s}


class ActiveObservationPlanner:
    """Plan a bounded, dynamically reachable observation trajectory.

    The result remains an observation *recommendation*, never a vehicle
    command. Every selected observation point lies on a collision-free
    straight segment that can be reached within the policy horizon.
    """
    def __init__(self, *, minimum_radius_m=1.5, maximum_radius_m=6., desired_parallax_deg=12.,
                 maximum_speed_mps=3., maximum_acceleration_mps2=2., planning_horizon_s=3., trajectory_steps=8):
        if not 0 < minimum_radius_m <= maximum_radius_m or not 0 < desired_parallax_deg < 90:
            raise ValueError("invalid observation planner limits")
        if (not all(np.isfinite(value) and value > 0 for value in
                    (maximum_speed_mps, maximum_acceleration_mps2, planning_horizon_s))
                or not isinstance(trajectory_steps, int) or trajectory_steps < 2):
            raise ValueError("invalid observation dynamics limits")
        self.minimum_radius_m, self.maximum_radius_m = float(minimum_radius_m), float(maximum_radius_m)
        self.desired_parallax_deg = float(desired_parallax_deg)
        self.maximum_speed_mps = float(maximum_speed_mps)
        self.maximum_acceleration_mps2 = float(maximum_acceleration_mps2)
        self.planning_horizon_s, self.trajectory_steps = float(planning_horizon_s), trajectory_steps

    @staticmethod
    def _segment_clear(start, end, spheres):
        direction = end - start
        squared = float(direction @ direction)
        for sphere in spheres:
            fraction = 0. if squared < 1e-12 else float(np.clip((sphere[:3] - start) @ direction / squared, 0., 1.))
            if np.linalg.norm(start + fraction * direction - sphere[:3]) <= sphere[3]:
                return False
        return True

    def _reachable_time_s(self, distance_m):
        transition = self.maximum_speed_mps ** 2 / self.maximum_acceleration_mps2
        if distance_m <= transition:
            return 2. * math.sqrt(distance_m / self.maximum_acceleration_mps2)
        return 2. * self.maximum_speed_mps / self.maximum_acceleration_mps2 + (distance_m - transition) / self.maximum_speed_mps

    def recommend(self, platform_map_m, target_map_m, *, previous_view_map_m=None, gdop=None,
                  target_sigma_m=None, reprojection_error_px=None, visibility=None,
                  allowed_altitude_m=None, forbidden_circles=(), forbidden_spheres=()):
        platform = _vector(platform_map_m, 3, "platform position")
        target = _vector(target_map_m, 3, "target position")
        if gdop is not None and (not np.isfinite(gdop) or gdop <= 0):
            raise ValueError("GDOP must be positive when available")
        if target_sigma_m is not None and (not np.isfinite(target_sigma_m) or target_sigma_m < 0):
            raise ValueError("target sigma must be nonnegative when available")
        if reprojection_error_px is not None and (not np.isfinite(reprojection_error_px) or reprojection_error_px < 0):
            raise ValueError("reprojection error must be nonnegative when available")
        if visibility is not None and (not np.isfinite(visibility) or not 0. <= visibility <= 1.):
            raise ValueError("target visibility must be a probability when available")
        radius = np.clip(2. + 4. * (0. if target_sigma_m is None else target_sigma_m), self.minimum_radius_m, self.maximum_radius_m)
        if gdop is not None:
            radius = np.clip(radius * min(1.5, max(.75, gdop / 2.)), self.minimum_radius_m, self.maximum_radius_m)
        if reprojection_error_px is not None:
            # A large pixel residual warrants a closer legal view for better
            # pixel geometry; the minimum radius remains a safety boundary.
            radius = np.clip(radius * (1. - .25 * min(1., reprojection_error_px / 4.)),
                             self.minimum_radius_m, self.maximum_radius_m)
        if visibility is not None:
            # With poor visual evidence, favour a legal closer observation;
            # uncertainty/GDOP still prevents collapsing below the safe radius.
            radius = np.clip(radius * (.75 + .25 * visibility), self.minimum_radius_m, self.maximum_radius_m)
        altitude = platform[2] if allowed_altitude_m is None else float(allowed_altitude_m)
        if not np.isfinite(altitude):
            raise ValueError("recommended altitude must be finite")
        circles = []
        for circle in forbidden_circles:
            value = _vector(circle, 3, "forbidden circle [x,y,radius]")
            if value[2] <= 0:
                raise ValueError("forbidden circle radius must be positive")
            circles.append(value)
        spheres = []
        for sphere in forbidden_spheres:
            value = _vector(sphere, 4, "forbidden sphere [x,y,z,radius]")
            if value[3] <= 0:
                raise ValueError("forbidden sphere radius must be positive")
            spheres.append(value)
        current = platform[:2] - target[:2]
        current_angle = math.atan2(current[1], current[0]) if np.linalg.norm(current) > 1e-6 else 0.
        candidates = current_angle + np.linspace(math.radians(35), math.radians(325), 9)
        best = None
        previous = None if previous_view_map_m is None else _vector(previous_view_map_m, 3, "previous view") - target
        for angle in candidates:
            point = target + np.array([radius * math.cos(angle), radius * math.sin(angle), altitude - target[2]])
            if any(np.linalg.norm(point[:2] - circle[:2]) <= circle[2] for circle in circles):
                continue
            baseline = np.linalg.norm(point - platform)
            travel_time_s = self._reachable_time_s(baseline)
            if travel_time_s > self.planning_horizon_s or not self._segment_clear(platform, point, spheres):
                continue
            parallax = 0.
            if previous is not None and np.linalg.norm(previous) > 1e-6:
                view = point - target
                parallax = math.degrees(math.acos(np.clip(np.dot(previous, view) / (np.linalg.norm(previous) * np.linalg.norm(view)), -1., 1.)))
            score = (min(1., baseline / radius) + min(1., parallax / self.desired_parallax_deg)
                     - .15 * travel_time_s / self.planning_horizon_s)
            if visibility is not None:
                score += .25 * visibility
            if gdop is not None:
                score /= max(1., gdop)
            if best is None or score > best[0]:
                best = score, point, angle, parallax, travel_time_s
        if best is None:
            raise ValueError("all observation candidates violate forbidden regions")
        score, point, angle, parallax, travel_time_s = best
        reason = "recover_visibility" if visibility is not None and visibility < .5 else (
            "reduce_reprojection_error" if reprojection_error_px is not None and reprojection_error_px > 2. else
            "increase_parallax" if parallax < self.desired_parallax_deg else "maintain_observability")
        trajectory = np.linspace(platform, point, self.trajectory_steps + 1)[1:]
        return ObservationRecommendation(point, math.atan2(target[1] - point[1], target[0] - point[0]), float(radius), float(score), reason,
                                         trajectory, float(travel_time_s))
