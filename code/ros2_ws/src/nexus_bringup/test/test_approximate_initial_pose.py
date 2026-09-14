"""Synthetic geometry and actual DDS wiring; no hardware precision claim."""

import json
from pathlib import Path
import time

import numpy as np
import pytest
import yaml

from nexus_bringup.approximate_initial_pose_node import initial_pose


def config():
    result = yaml.safe_load((Path(__file__).parents[1] / 'config/platform_live_approximate.yaml').read_text())
    result['initialization']['map_yaw_deg'] = 0.
    result['initialization']['use_fc_attitude'] = False
    result['initialization']['estimate_stationary_bias'] = True
    result['uwb']['tag_body_m'] = [.03, .02, .1]
    result['imu']['initial_gyro_bias_rps'] = [0., 0., 0.]
    result['imu']['initial_accel_bias_mps2'] = [0., 0., 0.]
    return result


def measurement(c):
    position = np.array([2., 2., .13])
    anchors = np.array(list(c['uwb']['anchors_map_m'].values()))
    return {'anchor_ids': ['1', '2', '3', '4'],
            'ranges_m': np.linalg.norm(anchors - position, axis=1).tolist()}


def test_initial_pose_compensates_lever_arm_without_faking_height():
    c = config()
    position, quaternion, rms = initial_pose(c, measurement(c), [0., 0., 9.80665])
    np.testing.assert_allclose(position, [1.97, 1.98, .03], atol=1e-5)
    np.testing.assert_allclose(quaternion, [0., 0., 0., 1.], atol=1e-6)
    assert rms < 1e-6
    with pytest.raises(ValueError):
        initial_pose(c, dict(measurement(c), ranges_m=[None] * 4), [0., 0., 9.80665])


@pytest.mark.parametrize("recovery", ["restart", "imu_gap", "moving_imu_gap", "moving_constraint_loss"])
def test_real_ros_raw_to_mean_to_initialized_platform(tmp_path, recovery):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from nexus_bringup.approximate_initial_pose_node import ApproximateInitialPoseNode
    from nexus_fusion_localization.platform_node import PlatformLocalizationNode
    from nexus_pi_readonly_ingress.range_smoothing_node import RangeSmoothingNode

    c = config()
    moving = recovery.startswith('moving_')
    c['initialization']['use_fc_attitude'] = moving
    c['initialization']['estimate_stationary_bias'] = not moving
    path = tmp_path / 'approximate.yaml'
    path.write_text(yaml.safe_dump(c))
    rclpy.init(args=['--ros-args', '-p', f'calibration_file:={path}', '-p',
                    'allow_approximate_calibration:=true', '-p',
                    'output_topic:=/nexus/uwb/startup_statistics', '-p', 'use_sim_time:=true'])
    nodes = [ApproximateInitialPoseNode(), PlatformLocalizationNode(), RangeSmoothingNode(), Node('initialization_test_driver')]
    initializer, platform, smoother, driver = nodes
    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    raw_imu = driver.create_publisher(Imu, '/imu_global_001', 100)
    raw_range = driver.create_publisher(String, '/nexus/uwb/ranges', 20)
    attitude_pub = driver.create_publisher(Odometry, '/nexus/pi/odom_aligned', 100)
    current_range = driver.create_publisher(String, '/nexus/pi/ranges_aligned', 20)
    clock_pub = driver.create_publisher(Clock, "/clock", 10)
    simulated_ns = 10_000_000_000
    outputs = []
    driver.create_subscription(Odometry, '/nexus/platform/odom', outputs.append, 20)

    def wait(predicate):
        deadline = time.monotonic() + 5
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.001)
        assert predicate(), 'DDS initialization condition timed out'

    def advance(delta_ns=5_000_000):
        nonlocal simulated_ns
        simulated_ns += delta_ns
        clock = Clock()
        clock.clock.sec, clock.clock.nanosec = divmod(simulated_ns, 10**9)
        clock_pub.publish(clock)
        wait(lambda: all(node.get_clock().now().nanoseconds == simulated_ns for node in nodes))
        return simulated_ns

    try:
        wait(lambda: clock_pub.get_subscription_count() == len(nodes))
        advance()
        wait(lambda: initializer.request_id is not None)
        wait(lambda: raw_imu.get_subscription_count() > 0 and raw_range.get_subscription_count() >= 2
             and initializer.pose_publisher.get_subscription_count() > 0
             and smoother.publisher.get_subscription_count() > 0)
        for i in range(20 if moving else 205):
            stamp = advance()
            imu = Imu()
            imu.header.stamp.sec, imu.header.stamp.nanosec = divmod(stamp, 10**9)
            imu.header.frame_id = c['imu']['frame_id']
            imu.linear_acceleration.z = 9.80665
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = .012, -.006, .02
            if moving:
                imu.angular_velocity.z = .2
            raw_imu.publish(imu)
            wait(lambda: initializer.samples and initializer.samples[-1][0] == stamp)
            packet = dict(measurement(c), unit='m', frame_id='map', schema_version=1, tag_id=2,
                          sample_timestamp_ns=stamp, variances_m2=[.01] * 4)
            if moving:
                attitude = Odometry()
                attitude.header.stamp.sec, attitude.header.stamp.nanosec = divmod(stamp, 10**9)
                attitude.header.frame_id = 'map'
                attitude.pose.pose.orientation.w = 1.
                attitude_pub.publish(attitude)
                wait(lambda: initializer.latest_attitude is not None and initializer.latest_attitude[0] == stamp)
                current_range.publish(String(data=json.dumps(packet)))
            raw_range.publish(String(data=json.dumps(packet)))
            wait(lambda: smoother.window.rows and smoother.window.rows[-1][0] == stamp)
        wait(lambda: initializer.initialized and len(outputs) > 0)
        assert smoother.last_result['window_samples'] == (20 if moving else 200)
        position = outputs[0].pose.pose.position
        np.testing.assert_allclose([position.x, position.y, position.z], [1.97, 1.98, .03], atol=1e-4)
        assert outputs[0].header.frame_id == 'map'
        assert platform._last_imu_ns > 0
        np.testing.assert_allclose(next(iter(platform.estimator.states.values())).gyro_bias_rps,
                                   ([0., 0., 0.] if moving else [.012, -.006, .02]), atol=1e-4)
        old_request = initializer.request_id
        if recovery == "restart":
            executor.remove_node(platform)
            platform.destroy_node()
            nodes.remove(platform)
            platform = PlatformLocalizationNode()
            nodes.append(platform)
            executor.add_node(platform)
            wait(lambda: clock_pub.get_subscription_count() == len(nodes))
            advance(100_000_000)
        elif recovery == 'moving_constraint_loss':
            # Advance through continuous moving IMU without external observations.
            for _ in range(320):
                stamp = advance()
                imu.header.stamp.sec, imu.header.stamp.nanosec = divmod(stamp, 10**9)
                raw_imu.publish(imu)
                wait(lambda: platform._last_imu_ns == stamp)
        else:
            stamp = advance(80_000_000)
            imu.header.stamp.sec, imu.header.stamp.nanosec = divmod(stamp, 10**9)
            raw_imu.publish(imu)
        wait(lambda: initializer.request_id != old_request)
        for i in range(20 if moving else 205):
            stamp = advance()
            imu = Imu()
            imu.header.stamp.sec, imu.header.stamp.nanosec = divmod(stamp, 10**9)
            imu.header.frame_id = c['imu']['frame_id']
            imu.linear_acceleration.z = 9.80665
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = .012, -.006, .02
            if moving:
                imu.angular_velocity.z = .2
            raw_imu.publish(imu)
            wait(lambda: initializer.samples and initializer.samples[-1][0] == stamp)
            packet = dict(measurement(c), unit='m', frame_id='map', schema_version=1, tag_id=2,
                          sample_timestamp_ns=stamp, variances_m2=[.01] * 4)
            if moving:
                attitude = Odometry()
                attitude.header.stamp.sec, attitude.header.stamp.nanosec = divmod(stamp, 10**9)
                attitude.header.frame_id = 'map'
                attitude.pose.pose.orientation.w = 1.
                attitude_pub.publish(attitude)
                wait(lambda: initializer.latest_attitude is not None and initializer.latest_attitude[0] == stamp)
                current_range.publish(String(data=json.dumps(packet)))
            raw_range.publish(String(data=json.dumps(packet)))
            wait(lambda: smoother.window.rows and smoother.window.rows[-1][0] == stamp)
        wait(lambda: initializer.initialized and bool(platform.estimator.states))
        assert initializer.request_id != old_request
    finally:
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()
