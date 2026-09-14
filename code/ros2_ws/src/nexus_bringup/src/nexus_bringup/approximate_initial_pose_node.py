#!/usr/bin/env python3
"""Read-only FC IMU relay and explicitly approximate stationary initialization."""

from collections import deque
import json

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from nexus_fusion_localization.platform_node import load_platform_calibration


def initial_pose(calibration, ranges, acceleration, attitude=None):
    """Estimate tag position and gravity attitude, using a disclosed yaw prior."""
    anchors = np.asarray([calibration['uwb']['anchors_map_m'][str(i)]
                          for i in ranges['anchor_ids']], dtype=float)
    distances = np.asarray(ranges['ranges_m'], dtype=float)
    if len(distances) < 4 or not np.isfinite(distances).all() or np.any(distances <= 0):
        raise ValueError('four finite positive mean ranges required for initialization')
    settings = calibration['initialization']
    yaw = np.deg2rad(settings['map_yaw_deg'])
    if attitude is None:
        acceleration = np.asarray(acceleration, dtype=float)
        if not 7 < np.linalg.norm(acceleration) < 12:
            raise ValueError('gravity magnitude is unavailable')
        roll = np.arctan2(acceleration[1], acceleration[2])
        pitch = np.arctan2(-acceleration[0], np.hypot(acceleration[1], acceleration[2]))
        rotation = Rotation.from_euler('xyz', [roll, pitch, yaw])
    else:
        rotation = Rotation.from_euler('z', yaw) * Rotation.from_quat(attitude)
    seed = np.r_[anchors[:, :2].mean(axis=0), settings['tag_height_seed_m']]
    # The known installation places the resting tag below the overhead anchors.
    lower, upper = settings['tag_height_bounds_m']
    result = least_squares(lambda p: np.linalg.norm(p - anchors, axis=1) - distances,
                           seed, bounds=([-np.inf, -np.inf, lower], [np.inf, np.inf, upper]),
                           loss='soft_l1', f_scale=.1)
    if not result.success or not np.isfinite(result.x).all():
        raise ValueError('range initialization solver failed')
    position = result.x - rotation.apply(calibration['uwb']['tag_body_m'])
    return position, rotation.as_quat(), float(np.sqrt(np.mean(result.fun ** 2)))


class ApproximateInitialPoseNode(Node):
    def __init__(self):
        super().__init__('nexus_approximate_initial_pose')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('allow_approximate_calibration', False)
        self.declare_parameter('imu_input_topic', '/imu_global_001')
        self.declare_parameter('statistics_topic', '/nexus/uwb/startup_statistics')
        self.calibration = load_platform_calibration(self.get_parameter('calibration_file').value,
            allow_approximate=self.get_parameter('allow_approximate_calibration').value)
        if not self.get_parameter('allow_approximate_calibration').value:
            raise ValueError('approximate initialization requires explicit opt-in')
        source = self.get_parameter('imu_input_topic').value
        if source == '/nexus/fcu/imu':
            raise ValueError('relay source must differ from its output')
        self.imu_publisher = self.create_publisher(Imu, '/nexus/fcu/imu', 100)
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/nexus/platform/initial_pose', 10)
        self.status_publisher = self.create_publisher(String, '/nexus/platform/initialization_status', 10)
        self.create_subscription(Imu, source, self.imu,
                                 QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(String, self.get_parameter('statistics_topic').value, self.ranges, 20)
        self.create_subscription(String, '/nexus/platform/status', self.platform_status, 20)
        self.create_subscription(String, '/nexus/platform/initialization_request', self.request, 10)
        self.use_fc_attitude = self.calibration.get('initialization', {}).get('use_fc_attitude', False)
        self.latest_attitude = None
        if self.use_fc_attitude:
            self.create_subscription(Odometry, '/nexus/pi/odom_aligned', self.attitude, 100)
            self.create_subscription(String, '/nexus/pi/ranges_aligned', self.moving_ranges, 20)
        self.latest_statistics = None
        self.request_id = None
        self.minimum_stamp = 0
        self.samples = deque(maxlen=200)
        self.initialized = False
        self.sent_stamp = 0

    def status(self, reason, **values):
        self.status_publisher.publish(String(data=json.dumps({
            'reason': reason, 'calibration_status': 'experimental_approximation',
            'approximations': self.calibration['approximations'], **values}, allow_nan=False)))

    def request(self, msg):
        packet = json.loads(msg.data)
        if packet['request_id'] != self.request_id:
            self.request_id = packet['request_id']
            self.minimum_stamp = int(packet['minimum_sample_timestamp_ns'])
            self.initialized = False
            self.sent_stamp = 0
            self.samples.clear()
            self.status('waiting_stationary_reinitialization', request_id=self.request_id)
            if self.latest_statistics is not None:
                self.ranges(self.latest_statistics)

    def platform_status(self, msg):
        packet = json.loads(msg.data)
        if (packet.get('reason') == 'initialized' and packet.get('valid') and
                packet.get('initialization_request_id') == self.request_id):
            self.initialized = True

    def imu(self, msg):
        # FC bridge has already converted vendor units and axes. Never double-convert.
        self.imu_publisher.publish(msg)
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        values = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                  msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        if (msg.header.frame_id == self.calibration['imu']['frame_id'] and np.isfinite(values).all()
                and msg.linear_acceleration_covariance[0] != -1 and msg.angular_velocity_covariance[0] != -1):
            if self.samples and (stamp <= self.samples[-1][0] or stamp - self.samples[-1][0] > 50_000_000):
                self.samples.clear()
            self.samples.append((stamp, values))

    def attitude(self, msg):
        q = msg.pose.pose.orientation
        quaternion = np.array([q.x, q.y, q.z, q.w])
        if msg.header.frame_id == 'map' and np.isfinite(quaternion).all() and abs(np.linalg.norm(quaternion) - 1.) < .001:
            self.latest_attitude = (msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec, quaternion)

    def moving_ranges(self, msg):
        if self.initialized or self.request_id is None or self.latest_attitude is None:
            return
        try:
            packet = json.loads(msg.data)
            if packet.get('schema_version') != 1 or packet.get('unit') != 'm' or packet.get('frame_id') != 'map':
                raise ValueError('moving initialization requires map ranges in meters')
            if len(set(packet['anchor_ids'])) != len(packet['anchor_ids']):
                raise ValueError('duplicate initialization anchors')
            stamp = packet['sample_timestamp_ns']
            now = self.get_clock().now().nanoseconds
            if stamp < self.minimum_stamp or not 0 <= now - stamp < 300_000_000:
                raise ValueError('moving initialization range stale')
            if not 0 <= now - self.latest_attitude[0] < 300_000_000 or abs(stamp-self.latest_attitude[0]) > 200_000_000:
                raise ValueError('moving initialization attitude stale or unpaired')
            if not self.samples or not 0 <= now-self.samples[-1][0] < 300_000_000:
                raise ValueError('moving initialization IMU unavailable')
            if self.sent_stamp and now-self.sent_stamp < 2_000_000_000:
                return
            position, quaternion, residual = initial_pose(self.calibration, packet, None, self.latest_attitude[1])
            self.publish_pose(stamp, position, quaternion, residual, 'fc_attitude_current_ranges')
        except (ValueError, TypeError, KeyError) as error:
            self.status('initialization_waiting:' + str(error))

    def publish_pose(self, stamp, position, quaternion, residual, source):
        output = PoseWithCovarianceStamped()
        output.header.frame_id = 'map'
        output.header.stamp.sec, output.header.stamp.nanosec = divmod(stamp, 10**9)
        output.pose.pose.position.x, output.pose.pose.position.y, output.pose.pose.position.z = position.tolist()
        (output.pose.pose.orientation.x, output.pose.pose.orientation.y,
         output.pose.pose.orientation.z, output.pose.pose.orientation.w) = quaternion.tolist()
        settings = self.calibration['initialization']
        covariance = np.diag([settings['position_sigma_m'] ** 2] * 3 + [np.deg2rad(settings['attitude_sigma_deg']) ** 2] * 3)
        output.pose.covariance = covariance.ravel().tolist()
        self.pose_publisher.publish(output)
        self.sent_stamp = self.get_clock().now().nanoseconds
        self.status('approximate_pose_sent', range_rms_m=residual, position_m=position.tolist(), source=source)

    def ranges(self, msg):
        if self.use_fc_attitude:
            return
        self.latest_statistics = msg
        if self.initialized:
            return
        try:
            packet = json.loads(msg.data)
            if (self.request_id is None or packet.get('stale') or not packet.get('ready') or len(self.samples) < 200
                    or packet.get('window_start_ns', 0) < self.minimum_stamp):
                self.status('waiting_stationary_window', window_samples=packet.get('window_samples', 0),
                            imu_samples=len(self.samples), request_received=self.request_id is not None,
                            minimum_sample_timestamp_ns=self.minimum_stamp)
                return
            stamp = packet['sample_timestamp_ns']
            now = self.get_clock().now().nanoseconds
            if not 0 <= now - stamp < 300_000_000 or not 0 <= now - self.samples[-1][0] < 300_000_000:
                raise ValueError('initialization inputs stale')
            values = np.array([row for _, row in self.samples])
            bias = np.asarray(self.calibration['imu'].get('initial_gyro_bias_rps', [0, 0, 0]))
            if np.max(values[:, :3].std(axis=0)) > .2 or np.max(np.abs(values[:, 3:] - bias)) > .05:
                raise ValueError('hold platform stationary for initialization')
            if self.sent_stamp and now - self.sent_stamp < 2_000_000_000:
                return
            if not self.pose_publisher.get_subscription_count():
                return
            position, quaternion, residual = initial_pose(self.calibration, packet, values[:, :3].mean(axis=0))
            self.publish_pose(stamp, position, quaternion, residual, 'stationary_window')
        except (ValueError, TypeError, KeyError) as error:
            self.status('initialization_waiting:' + str(error))


def main():
    rclpy.init()
    node = ApproximateInitialPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
