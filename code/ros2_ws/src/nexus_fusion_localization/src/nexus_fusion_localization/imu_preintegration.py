"""Midpoint IMU preintegration with bias Jacobians and measurement covariance."""

from dataclasses import dataclass

import numpy as np

from .platform_state import BodyState, exp_so3, inverse_right_jacobian_so3, log_so3, right_jacobian_so3, skew, vector


@dataclass(frozen=True)
class ImuReading:
    stamp_ns: int
    acceleration_mps2: np.ndarray
    angular_velocity_rps: np.ndarray

    def __post_init__(self):
        if isinstance(self.stamp_ns, bool) or int(self.stamp_ns) != self.stamp_ns or self.stamp_ns <= 0:
            raise ValueError("IMU timestamp must be a positive integer")
        object.__setattr__(self, "acceleration_mps2", vector(self.acceleration_mps2, 3, "specific force"))
        object.__setattr__(self, "angular_velocity_rps", vector(self.angular_velocity_rps, 3, "angular velocity"))


@dataclass(frozen=True)
class ImuNoise:
    accel_density: float = .08
    gyro_density: float = .004
    accel_bias_walk: float = .001
    gyro_bias_walk: float = .0001
    maximum_gap_s: float = .05

    def __post_init__(self):
        values = tuple(self.__dict__.values())
        if not np.all(np.isfinite(values)) or min(values) <= 0:
            raise ValueError("IMU noise densities and maximum gap must be positive")


class ImuBuffer:
    def __init__(self):
        self.samples = []

    def add(self, sample):
        if not isinstance(sample, ImuReading):
            raise TypeError("IMU buffer requires ImuReading")
        if self.samples and sample.stamp_ns <= self.samples[-1].stamp_ns:
            raise ValueError("nonmonotonic IMU sample time")
        self.samples.append(sample)

    def interval(self, start_ns, end_ns, maximum_gap_s):
        if (start_ns >= end_ns or len(self.samples) < 2
                or self.samples[0].stamp_ns > start_ns or self.samples[-1].stamp_ns < end_ns):
            raise ValueError("IMU samples do not bracket the complete integration interval")
        stamps = np.array([sample.stamp_ns for sample in self.samples], dtype=np.int64)

        def boundary(stamp):
            index = int(np.searchsorted(stamps, stamp))
            if stamps[index] == stamp:
                return self.samples[index]
            before, after = self.samples[index - 1], self.samples[index]
            if (after.stamp_ns - before.stamp_ns) / 1e9 > maximum_gap_s:
                raise ValueError("IMU gap exceeds maximum_gap_s")
            fraction = (stamp - before.stamp_ns) / (after.stamp_ns - before.stamp_ns)
            return ImuReading(stamp,
                              before.acceleration_mps2 * (1 - fraction) + after.acceleration_mps2 * fraction,
                              before.angular_velocity_rps * (1 - fraction) + after.angular_velocity_rps * fraction)

        result = [boundary(start_ns)]
        result.extend(sample for sample in self.samples if start_ns < sample.stamp_ns < end_ns)
        result.append(boundary(end_ns))
        if any((b.stamp_ns - a.stamp_ns) / 1e9 > maximum_gap_s for a, b in zip(result, result[1:])):
            raise ValueError("IMU gap exceeds maximum_gap_s")
        return result

    def discard_before(self, stamp_ns):
        # Keep the bracketing sample immediately preceding the active window.
        index = max(0, int(np.searchsorted([s.stamp_ns for s in self.samples], stamp_ns)) - 1)
        self.samples = self.samples[index:]


def _integrate(samples, accel_bias, gyro_bias, noise=None):
    rotation, velocity, position, covariance = np.eye(3), np.zeros(3), np.zeros(3), np.zeros((9, 9))
    for first, second in zip(samples, samples[1:]):
        dt = (second.stamp_ns - first.stamp_ns) / 1e9
        acceleration = (first.acceleration_mps2 + second.acceleration_mps2) / 2 - accel_bias
        omega = (first.angular_velocity_rps + second.angular_velocity_rps) / 2 - gyro_bias
        middle = rotation @ exp_so3(omega * dt / 2)
        force = middle @ acceleration
        if noise is not None:
            transition = np.eye(9)  # theta, delta_v, delta_p
            transition[:3, :3] = exp_so3(-omega * dt)
            transition[3:6, :3] = -middle @ skew(acceleration) * dt
            transition[6:9, :3] = -middle @ skew(acceleration) * dt ** 2 / 2
            transition[6:9, 3:6] = np.eye(3) * dt
            process = np.zeros((9, 9))
            process[:3, :3] = np.eye(3) * noise.gyro_density ** 2 * dt
            process[3:6, 3:6] = np.eye(3) * noise.accel_density ** 2 * dt
            process[6:9, 6:9] = np.eye(3) * noise.accel_density ** 2 * dt ** 3 / 3
            process[3:6, 6:9] = process[6:9, 3:6] = np.eye(3) * noise.accel_density ** 2 * dt ** 2 / 2
            covariance = transition @ covariance @ transition.T + process
        position += velocity * dt + force * dt ** 2 / 2
        velocity += force * dt
        rotation = rotation @ exp_so3(omega * dt)
    return rotation, velocity, position, covariance


@dataclass(frozen=True)
class PreintegratedImu:
    start_ns: int
    end_ns: int
    delta_rotation: np.ndarray
    delta_velocity: np.ndarray
    delta_position: np.ndarray
    covariance: np.ndarray
    bias_jacobian: np.ndarray  # rows theta/v/p, columns ba/bg
    accel_bias: np.ndarray
    gyro_bias: np.ndarray
    noise: ImuNoise

    def __post_init__(self):
        object.__setattr__(self, "whitener", np.linalg.solve(np.linalg.cholesky(self.covariance), np.eye(9)))
        object.__setattr__(self, "bias_walk_inverse_sigma", np.repeat(
            [1 / (self.noise.accel_bias_walk * np.sqrt(self.dt)),
             1 / (self.noise.gyro_bias_walk * np.sqrt(self.dt))], 3))

    @property
    def dt(self):
        return (self.end_ns - self.start_ns) / 1e9

    @classmethod
    def from_buffer(cls, buffer, start_ns, end_ns, accel_bias, gyro_bias, noise):
        accel_bias, gyro_bias = vector(accel_bias, 3, "accel bias"), vector(gyro_bias, 3, "gyro bias")
        samples = buffer.interval(start_ns, end_ns, noise.maximum_gap_s)
        rotation, velocity, position, covariance = _integrate(samples, accel_bias, gyro_bias, noise)
        jacobian = np.zeros((9, 6))
        epsilon = 1e-5
        biases = np.r_[accel_bias, gyro_bias]
        for axis in range(6):
            offset = np.zeros(6)
            offset[axis] = epsilon
            plus, minus = biases + offset, biases - offset
            rp, vp, pp, _ = _integrate(samples, plus[:3], plus[3:])
            rm, vm, pm, _ = _integrate(samples, minus[:3], minus[3:])
            jacobian[:, axis] = np.r_[log_so3(rotation.T @ rp) - log_so3(rotation.T @ rm),
                                      vp - vm, pp - pm] / (2 * epsilon)
        return cls(start_ns, end_ns, rotation, velocity, position, covariance,
                   jacobian, accel_bias, gyro_bias, noise)

    def corrected(self, state):
        shift = self.bias_jacobian @ np.concatenate((state.accel_bias_mps2 - self.accel_bias,
                                                    state.gyro_bias_rps - self.gyro_bias))
        return (self.delta_rotation @ exp_so3(shift[:3]), self.delta_velocity + shift[3:6],
                self.delta_position + shift[6:9])

    def predict(self, initial, gravity):
        if initial.stamp_ns != self.start_ns:
            raise ValueError("preintegration start does not match body state")
        rotation, velocity, position = self.corrected(initial)
        gravity = vector(gravity, 3, "map gravity")
        return BodyState(self.end_ns,
                         initial.position_m + initial.velocity_mps * self.dt + gravity * self.dt ** 2 / 2
                         + initial.rotation_map_body @ position,
                         initial.velocity_mps + gravity * self.dt + initial.rotation_map_body @ velocity,
                         initial.rotation_map_body @ rotation,
                         initial.accel_bias_mps2, initial.gyro_bias_rps)

    def residual(self, first, second, gravity):
        rotation, velocity, position = self.corrected(first)
        dt, body_rotation = self.dt, first.rotation_map_body
        delta_position = body_rotation.T @ (
            second.position_m - first.position_m - first.velocity_mps * dt - gravity * dt ** 2 / 2) - position
        delta = np.concatenate((log_so3(rotation.T @ body_rotation.T @ second.rotation_map_body),
                                body_rotation.T @ (second.velocity_mps - first.velocity_mps - gravity * dt) - velocity,
                                delta_position))
        whitened = self.whitener @ delta
        bias_walk = np.concatenate((second.accel_bias_mps2 - first.accel_bias_mps2,
                                    second.gyro_bias_rps - first.gyro_bias_rps)) * self.bias_walk_inverse_sigma
        return np.concatenate((whitened, bias_walk))

    def jacobians(self, first, second, gravity):
        """Exact local derivatives of the bias-corrected preintegration residual."""
        rotation, _, _ = self.corrected(first)
        body_inverse = first.rotation_map_body.T
        angle = log_so3(rotation.T @ body_inverse @ second.rotation_map_body)
        left_log = inverse_right_jacobian_so3(-angle)
        bias_delta = np.concatenate((first.accel_bias_mps2 - self.accel_bias,
                                     first.gyro_bias_rps - self.gyro_bias))
        shift = self.bias_jacobian[:3] @ bias_delta
        left, right = np.zeros((15, 15)), np.zeros((15, 15))
        left[:3, 6:9] = -left_log @ rotation.T
        right[:3, 6:9] = inverse_right_jacobian_so3(angle)
        left[:3, 9:15] = -left_log @ right_jacobian_so3(shift) @ self.bias_jacobian[:3]
        left[3:6, 3:6], right[3:6, 3:6] = -body_inverse, body_inverse
        left[3:6, 6:9] = skew(body_inverse @ (second.velocity_mps - first.velocity_mps - gravity * self.dt))
        left[3:6, 9:15] = -self.bias_jacobian[3:6]
        left[6:9, :3], right[6:9, :3] = -body_inverse, body_inverse
        left[6:9, 3:6] = -body_inverse * self.dt
        displacement = second.position_m - first.position_m - first.velocity_mps * self.dt - gravity * self.dt ** 2 / 2
        left[6:9, 6:9] = skew(body_inverse @ displacement)
        left[6:9, 9:15] = -self.bias_jacobian[6:9]
        left[:9] = self.whitener @ left[:9]
        right[:9] = self.whitener @ right[:9]
        left[9:, 9:] = -np.diag(self.bias_walk_inverse_sigma)
        right[9:, 9:] = np.diag(self.bias_walk_inverse_sigma)
        return {self.start_ns: left, self.end_ns: right}

    def transition_matrix(self, initial):
        transition = np.eye(15)
        rotation, velocity, position = self.corrected(initial)
        transition[:3, 3:6] = np.eye(3) * self.dt
        transition[:3, 6:9] = -initial.rotation_map_body @ skew(position)
        transition[3:6, 6:9] = -initial.rotation_map_body @ skew(velocity)
        transition[6:9, 6:9] = rotation.T
        transition[:3, 9:15] = initial.rotation_map_body @ self.bias_jacobian[6:9]
        transition[3:6, 9:15] = initial.rotation_map_body @ self.bias_jacobian[3:6]
        transition[6:9, 9:15] = self.bias_jacobian[:3]
        return transition

    def predict_covariance(self, initial, covariance):
        transition = self.transition_matrix(initial)
        mapping = np.zeros((15, 9))
        mapping[6:9, :3] = np.eye(3)
        mapping[3:6, 3:6] = initial.rotation_map_body
        mapping[:3, 6:9] = initial.rotation_map_body
        process = mapping @ self.covariance @ mapping.T
        process[9:12, 9:12] = np.eye(3) * self.noise.accel_bias_walk ** 2 * self.dt
        process[12:15, 12:15] = np.eye(3) * self.noise.gyro_bias_walk ** 2 * self.dt
        result = transition @ covariance @ transition.T + process
        return (result + result.T) / 2
