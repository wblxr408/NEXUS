"""Bounded platform p/v/R/ba/bg graph with IMU, absolute and relative factors."""

from dataclasses import dataclass, field, replace
from time import perf_counter

import numpy as np
from scipy.optimize import least_squares
from scipy.stats import chi2

from .imu_preintegration import ImuBuffer, ImuNoise, PreintegratedImu
from .marginalization import MarginalPrior, schur_square_root
from .platform_measurements import PlatformMeasurement
from .platform_state import BodyState, covariance_matrix, right_jacobian_so3, vector
from .robust_filter import innovation_gate


@dataclass(frozen=True)
class PlatformConfig:
    window_size: int = 6
    max_nfev: int = 40
    irls_iterations: int = 2
    kernel: str = "huber"
    kernel_scale: float = 3.0
    prediction_horizon_s: float = .5
    maximum_speed_mps: float = 20.0
    maximum_specific_force_mps2: float = 100.0
    maximum_angular_rate_rps: float = 20.0
    gravity: tuple = (0., 0., -9.80665)
    imu_noise: ImuNoise = field(default_factory=ImuNoise)

    def __post_init__(self):
        if (self.window_size < 2 or self.max_nfev < 1 or self.irls_iterations < 1
                or self.kernel not in {"linear", "huber", "cauchy", "tukey", "switchable"}
                or not np.all(np.isfinite((self.kernel_scale, self.prediction_horizon_s, self.maximum_speed_mps,
                                          self.maximum_specific_force_mps2, self.maximum_angular_rate_rps)))
                or min(self.kernel_scale, self.prediction_horizon_s, self.maximum_speed_mps,
                       self.maximum_specific_force_mps2, self.maximum_angular_rate_rps) <= 0):
            raise ValueError("invalid platform window configuration")
        vector(self.gravity, 3, "map gravity")


@dataclass(frozen=True)
class PlatformUpdate:
    state: BodyState | None
    covariance: np.ndarray | None
    valid: bool
    reason: str
    prediction_only: bool
    diagnostics: tuple
    window_stamps: tuple
    runtime_ms: float


@dataclass(frozen=True)
class _ImuFactor:
    integrated: PreintegratedImu
    gravity: np.ndarray
    covariance_scale: float = 1.0

    @property
    def stamps(self):
        return self.integrated.start_ns, self.integrated.end_ns

    def residual(self, states):
        first, second = (states[stamp] for stamp in self.stamps)
        residual = self.integrated.residual(first, second, self.gravity)
        residual[:9] /= np.sqrt(self.covariance_scale)
        return residual

    def jacobians(self, states):
        first, second = (states[stamp] for stamp in self.stamps)
        result = self.integrated.jacobians(first, second, self.gravity)
        for matrix in result.values():
            matrix[:9] /= np.sqrt(self.covariance_scale)
        return result


def robust_weight(norm, kernel, scale):
    """Square root IRLS weight of an entire whitened observation block."""
    if kernel == "linear" or norm == 0:
        return 1.0
    ratio = norm / scale
    if kernel == "huber":
        return min(1., np.sqrt(1. / ratio))
    if kernel == "cauchy":
        return 1. / np.sqrt(1. + ratio ** 2)
    if kernel == "tukey":
        return max(0., 1. - ratio ** 2)
    if kernel == "switchable":
        # Closed-form switch s = lambda / (lambda + ||r||²), lambda=scale².
        return 1. / (1. + ratio ** 2)
    raise ValueError("unknown robust kernel")


class PlatformWindow:
    def __init__(self, config=None):
        self.config = config or PlatformConfig()
        self.imu = ImuBuffer()
        self.states = {}
        self.factors = []
        self.prior = None
        self.joint_covariance = None
        self.measurement_keys = set()
        self.last_external_ns = 0
        self.last_source_stamps = {}
        self.marginalization_count = 0

    def initialize(self, state, covariance, source="initial_pose"):
        if not isinstance(state, BodyState):
            raise TypeError("platform initialization requires BodyState")
        matrix = covariance_matrix(covariance, 15)
        self.states = {state.stamp_ns: state}
        self.factors, self.measurement_keys = [], set()
        self.prior = MarginalPrior((state,), np.linalg.solve(np.linalg.cholesky(matrix), np.eye(15)), np.zeros(15))
        self.joint_covariance = matrix
        self.last_external_ns = state.stamp_ns
        self.last_source_stamps = {source: state.stamp_ns}
        self.marginalization_count = 0
        self.imu.discard_before(state.stamp_ns)

    def add_imu(self, reading):
        if (np.linalg.norm(reading.acceleration_mps2) > self.config.maximum_specific_force_mps2
                or np.linalg.norm(reading.angular_velocity_rps) > self.config.maximum_angular_rate_rps):
            raise ValueError("IMU acceleration_or_angular_rate_limit; check SI units")
        self.imu.add(reading)

    def _weights(self, factors, states):
        return [robust_weight(float(np.linalg.norm(factor.residual(states))), self.config.kernel,
                              self.config.kernel_scale) if isinstance(factor, PlatformMeasurement) else 1.
                for factor in factors]

    @staticmethod
    def _stack(states, prior, factors, weights):
        blocks = [prior.residual(states)] if prior is not None else []
        blocks.extend(weight * factor.residual(states) for factor, weight in zip(factors, weights))
        return np.concatenate(blocks)

    @staticmethod
    def _linearize(states, prior, factors, weights):
        stamps = tuple(states)
        rows, matrices = [], []
        blocks = ([(prior, 1.)] if prior is not None else []) + list(zip(factors, weights))
        for factor, weight in blocks:
            residual = factor.residual(states) * weight
            matrix = np.zeros((len(residual), len(stamps) * 15))
            for stamp, block in factor.jacobians(states).items():
                index = stamps.index(stamp)
                matrix[:, index * 15:(index + 1) * 15] = block * weight
            rows.append(residual)
            matrices.append(matrix)
        return np.concatenate(rows), np.vstack(matrices)

    def _solve(self, states, prior, factors):
        stamps = tuple(states)
        current = states
        result = None
        for _ in range(self.config.irls_iterations):
            for index, factor in enumerate(factors):
                if isinstance(factor, _ImuFactor):
                    initial = current[factor.integrated.start_ns]
                    if (np.linalg.norm(initial.accel_bias_mps2 - factor.integrated.accel_bias) > .05
                            or np.linalg.norm(initial.gyro_bias_rps - factor.integrated.gyro_bias) > .005):
                        # Large bias corrections exceed the first-order cache.
                        integrated = PreintegratedImu.from_buffer(
                            self.imu, *factor.stamps, initial.accel_bias_mps2,
                            initial.gyro_bias_rps, self.config.imu_noise)
                        factors[index] = replace(factor, integrated=integrated)
            weights = self._weights(factors, current)
            references = current

            def decode(delta):
                return {stamp: references[stamp].retract(delta[i * 15:(i + 1) * 15])
                        for i, stamp in enumerate(stamps)}

            cached_delta, cached_residual, cached_jacobian = None, None, None

            def evaluate(delta):
                nonlocal cached_delta, cached_residual, cached_jacobian
                if cached_delta is None or not np.array_equal(delta, cached_delta):
                    values, matrix = self._linearize(decode(delta), prior, factors, weights)
                    for index in range(len(stamps)):
                        angle_slice = slice(index * 15 + 6, index * 15 + 9)
                        matrix[:, angle_slice] = matrix[:, angle_slice] @ right_jacobian_so3(delta[angle_slice])
                    cached_delta, cached_residual, cached_jacobian = delta.copy(), values, matrix
                return cached_residual, cached_jacobian

            def residual(delta):
                return evaluate(delta)[0]

            def jacobian(delta):
                return evaluate(delta)[1]

            result = least_squares(residual, np.zeros(len(stamps) * 15), jac=jacobian,
                                   method="trf", loss="linear", max_nfev=self.config.max_nfev,
                                   x_scale="jac", ftol=1e-6, xtol=1e-7, gtol=1e-6)
            if not result.success or not np.all(np.isfinite(result.x)):
                raise ValueError(f"platform_solver_failed:{result.message}")
            current = decode(result.x)
        # Re-linearize in the returned states' own tangent spaces (rather than
        # using covariance in the pre-step rotation perturbation coordinates).
        weights = self._weights(factors, current)
        _, jacobian = self._linearize(current, prior, factors, weights)
        information = jacobian.T @ jacobian
        try:
            covariance = np.linalg.solve(information, np.eye(information.shape[0]))
            covariance_matrix((covariance + covariance.T) / 2, information.shape[0])
        except (ValueError, np.linalg.LinAlgError) as error:
            raise ValueError("platform_window_has_unobservable_state") from error
        return current, (covariance + covariance.T) / 2, weights

    def _gate(self, measurement, states, covariance):
        involved = {stamp: states[stamp] for stamp in measurement.stamps}
        residual = measurement.raw_residual(involved)
        blocks = measurement.raw_jacobians(involved)
        jacobian = np.hstack([blocks[stamp] for stamp in measurement.stamps])
        all_stamps = tuple(states)
        indices = np.concatenate([np.arange(all_stamps.index(stamp) * 15, (all_stamps.index(stamp) + 1) * 15)
                                  for stamp in measurement.stamps])
        innovation_covariance = jacobian @ covariance[np.ix_(indices, indices)] @ jacobian.T + measurement.covariance
        dof = measurement.degrees_of_freedom
        decision, d2, inflation = innovation_gate(residual, innovation_covariance,
                                                  chi2.ppf(.95, dof), chi2.ppf(.999, dof))
        return decision, d2, inflation

    def _marginalize(self, states, covariance, factors, prior):
        oldest = next(iter(states))
        outgoing = [factor for factor in factors if oldest in factor.stamps]
        retained = [factor for factor in factors if oldest not in factor.stamps]
        involved_stamps = set(prior.stamps if prior else ())
        for factor in outgoing:
            involved_stamps.update(factor.stamps)
        involved = {stamp: state for stamp, state in states.items() if stamp in involved_stamps}
        weights = self._weights(outgoing, involved)
        residual, jacobian = self._linearize(involved, prior, outgoing, weights)
        old_index = tuple(involved).index(oldest)
        matrix, offset, _ = schur_square_root(jacobian, residual, np.arange(old_index * 15, old_index * 15 + 15))
        references = tuple(state for stamp, state in involved.items() if stamp != oldest)
        new_prior = MarginalPrior(references, matrix, offset)
        remaining = {stamp: state for stamp, state in states.items() if stamp != oldest}
        return remaining, covariance[15:, 15:].copy(), retained, new_prior

    def _output(self, started, valid, reason, prediction_only, diagnostics=()):
        state = next(reversed(self.states.values())) if self.states else None
        covariance = self.joint_covariance[-15:, -15:].copy() if self.joint_covariance is not None else None
        return PlatformUpdate(state, covariance, valid, reason, prediction_only, tuple(diagnostics),
                              tuple(self.states), (perf_counter() - started) * 1000)

    def _insert_state(self, current, factors, covariance, stamp_ns, imu_covariance_scale):
        """Add one actual sample time and split its spanning preintegration edge."""
        old_stamps = tuple(current)
        left_stamp = max(stamp for stamp in old_stamps if stamp < stamp_ns)
        previous = current[left_stamp]
        right_stamp = next((stamp for stamp in old_stamps if stamp > stamp_ns), None)
        integrated = PreintegratedImu.from_buffer(
            self.imu, previous.stamp_ns, stamp_ns,
            previous.accel_bias_mps2, previous.gyro_bias_rps, self.config.imu_noise)
        predicted = integrated.predict(previous, np.asarray(self.config.gravity))
        transition = integrated.transition_matrix(previous)
        left_index = old_stamps.index(left_stamp)
        left_slice = slice(left_index * 15, (left_index + 1) * 15)
        cross = covariance[:, left_slice] @ transition.T
        predicted_covariance = integrated.predict_covariance(previous, covariance[left_slice, left_slice])
        if right_stamp is not None:
            old_index = next(index for index, factor in enumerate(factors)
                             if isinstance(factor, _ImuFactor) and factor.stamps == (left_stamp, right_stamp))
            old_edge = factors.pop(old_index)
            imu_covariance_scale = old_edge.covariance_scale
            following = PreintegratedImu.from_buffer(
                self.imu, stamp_ns, right_stamp, predicted.accel_bias_mps2,
                predicted.gyro_bias_rps, self.config.imu_noise)
            factors.append(_ImuFactor(following, np.asarray(self.config.gravity), imu_covariance_scale))
        if imu_covariance_scale > 1:
            propagated = transition @ covariance[left_slice, left_slice] @ transition.T
            added = predicted_covariance - propagated
            added[:9, :9] *= imu_covariance_scale
            predicted_covariance = propagated + added
        covariance = np.block([[covariance, cross], [cross.T, predicted_covariance]])
        current[stamp_ns] = predicted
        appended_order = tuple(current)
        current = dict(sorted(current.items()))
        permutation = np.concatenate([np.arange(appended_order.index(stamp) * 15, (appended_order.index(stamp) + 1) * 15)
                                      for stamp in current])
        covariance = covariance[np.ix_(permutation, permutation)]
        factors.append(_ImuFactor(integrated, np.asarray(self.config.gravity), imu_covariance_scale))
        return current, covariance

    def step(self, stamp_ns, measurements=(), *, imu_covariance_scale=1.0):
        started = perf_counter()
        if not self.states:
            return self._output(started, False, "platform_uninitialized", True)
        if not np.isfinite(imu_covariance_scale) or not 1 <= imu_covariance_scale <= 100:
            return self._output(started, False, "invalid_imu_covariance_scale", True)
        if any(not isinstance(item, PlatformMeasurement) or item.stamp_ns != stamp_ns for item in measurements):
            return self._output(started, False, "measurement_time_or_type_mismatch", True)
        current = dict(self.states)
        factors = list(self.factors)
        covariance = self.joint_covariance.copy()
        if stamp_ns < next(iter(current)):
            return self._output(started, False, "unrepresented_late_timestamp", True)
        try:
            needed_stamps = {stamp_ns}
            for measurement in measurements:
                if measurement.key not in self.measurement_keys and min(measurement.stamps) >= next(iter(current)):
                    needed_stamps.update(measurement.stamps)
            for sample_ns in sorted(needed_stamps - current.keys()):
                # Camera and UWB clocks share an epoch, not sampling instants.
                # Both endpoints of a visual edge need their actual states.
                current, covariance = self._insert_state(current, factors, covariance, sample_ns, imu_covariance_scale)
            diagnostics, accepted = [], []
            for measurement in measurements:
                if measurement.key in self.measurement_keys or any(item.key == measurement.key for item in accepted):
                    diagnostics.append({"source": measurement.source, "decision": "rejected", "reason": "duplicate_measurement"})
                    continue
                if not set(measurement.stamps).issubset(current):
                    diagnostics.append({"source": measurement.source, "decision": "rejected", "reason": "visual_reference_outside_window"})
                    continue
                try:
                    decision, d2, inflation = self._gate(measurement, current, covariance)
                except (ValueError, np.linalg.LinAlgError) as error:
                    diagnostics.append({"source": measurement.source, "decision": "rejected", "reason": str(error)})
                    continue
                diagnostics.append({"source": measurement.source, "decision": decision,
                                    "reason": "innovation_outlier" if decision == "rejected" else "", "d2": d2})
                if decision != "rejected":
                    accepted.append(replace(measurement, covariance_scale=min(100., measurement.covariance_scale * inflation)))
            if not accepted and stamp_ns <= next(reversed(self.states)):
                return self._output(started, False, "no_new_accepted_measurement", True, diagnostics)
            factors.extend(accepted)
            if accepted:
                current, covariance, weights = self._solve(current, self.prior, factors)
            else:
                weights = self._weights(factors, current)
            if np.linalg.norm(next(reversed(current.values())).velocity_mps) > self.config.maximum_speed_mps:
                raise ValueError("platform_speed_limit")
            effective_keys = {factor.key for factor, weight in zip(factors, weights)
                              if isinstance(factor, PlatformMeasurement) and weight > 1e-6}
            prior = self.prior
            marginalized = 0
            while len(current) > self.config.window_size:
                current, covariance, factors, prior = self._marginalize(current, covariance, factors, prior)
                marginalized += 1
        except (ValueError, np.linalg.LinAlgError) as error:
            return self._output(started, False, str(error), True)
        # Commit only after solve, validity and marginalization all succeeded.
        self.states, self.joint_covariance, self.factors, self.prior = current, covariance, factors, prior
        self.marginalization_count += marginalized
        fresh = [item for item in accepted if item.key in effective_keys]
        for item in fresh:
            self.last_external_ns = max(self.last_external_ns, item.stamp_ns)
            self.last_source_stamps[item.source] = max(item.stamp_ns, self.last_source_stamps.get(item.source, 0))
        self.measurement_keys = {factor.key for factor in factors if isinstance(factor, PlatformMeasurement)}
        self.imu.discard_before(next(iter(current)))
        last_stamp = next(reversed(current))
        timed_out = (last_stamp - self.last_external_ns) / 1e9 > self.config.prediction_horizon_s
        reason = "external_constraints_lost" if timed_out else "imu_prediction_only" if not fresh else ""
        return self._output(started, not timed_out, reason, not fresh, diagnostics)
