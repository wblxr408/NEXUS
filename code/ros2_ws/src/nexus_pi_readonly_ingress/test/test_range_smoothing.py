from nexus_pi_readonly_ingress.robust_range_window import RobustRangeWindow


def packet(index, values=None):
    return {'unit': 'm', 'tag_id': 2, 'anchor_ids': ['1', '2', '3', '4'],
            'sample_timestamp_ns': (index + 1) * 10**9,
            'ranges_m': values if values is not None else [5., 2., 4., 5.5]}


def test_outlier_rejection_and_window_eviction():
    window = RobustRangeWindow()
    for i in range(200):
        result = window.push(packet(i, [.06, 2., 4., 5.5] if i == 100 else None), i)
    assert result['ready']
    assert result['ranges_m'] == [5., 2., 4., 5.5]
    assert result['statistics'][0]['rejected'] == 1
    assert result['window_duration_s'] == 199
    for i in range(200, 401):
        result = window.push(packet(i, [6., 3., 5., 6.5]), i)
    assert result['window_samples'] == 200
    assert result['ranges_m'] == [6., 3., 5., 6.5]


def test_duplicate_does_not_fill_window_or_refresh_arrival():
    window = RobustRangeWindow()
    window.push(packet(0), 0)
    for _ in range(200):
        assert window.push(packet(0), 1) is None
    assert len(window.rows) == 1
    assert window.last_arrival == 0


def test_invalid_slots_and_reconnect_reset():
    window = RobustRangeWindow()
    result = window.push(packet(0, [None, 0, float('nan'), 655.35]), 0)
    assert result['ranges_m'] == [None] * 4
    result = window.push(packet(1), 10)
    assert result['window_samples'] == 1
    assert result['ranges_m'] == [5., 2., 4., 5.5]


def test_identity_change_does_not_mix_tags():
    window = RobustRangeWindow()
    window.push(packet(0), 0)
    other = dict(packet(1), tag_id=1)
    assert window.push(other, 1)['window_samples'] == 1


def test_ros_output_and_stale_status():
    import json
    import time

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    from nexus_pi_readonly_ingress.range_smoothing_node import RangeSmoothingNode

    rclpy.init(args=['--ros-args', '-p', 'input_topic:=/test_robust_range/input',
                    '-p', 'output_topic:=/test_robust_range/output'])
    smoother = RangeSmoothingNode()
    driver = Node('robust_range_test_driver')
    publisher = driver.create_publisher(String, '/test_robust_range/input', 20)
    received = []
    driver.create_subscription(String, '/test_robust_range/output',
                               lambda msg: received.append(json.loads(msg.data)), 20)

    def wait_until(predicate):
        deadline = time.monotonic() + 5
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(smoother, timeout_sec=.01)
            rclpy.spin_once(driver, timeout_sec=.01)
        assert predicate()

    try:
        wait_until(lambda: publisher.get_subscription_count() > 0 and
                   smoother.publisher.get_subscription_count() > 0)
        publisher.publish(String(data=json.dumps(packet(0))))
        wait_until(lambda: len(received) == 1)
        assert received[-1]['ranges_m'] == [5., 2., 4., 5.5]
        assert received[-1]['window_samples'] == 1
        smoother.window.last_arrival -= 6
        smoother.check_stale()
        wait_until(lambda: len(received) == 2)
        assert received[-1]['stale']
        assert received[-1]['ranges_m'] == [None] * 4
        publisher.publish(String(data=json.dumps(packet(1))))
        wait_until(lambda: len(received) == 3)
        assert not received[-1]['stale']
        assert received[-1]['window_samples'] == 1
    finally:
        smoother.destroy_node()
        driver.destroy_node()
        rclpy.shutdown()
