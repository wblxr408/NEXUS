#!/usr/bin/env python3
"""Opt-in real GPU/ROS chain with synthetic planar imagery and known camera poses."""

import argparse
import json
import os
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
import yaml

from nexus_msgs.msg import TargetKinematicState
from nexus_vision_localization.detection_node import ObjectDetectionNode
from nexus_vision_localization.superpoint_node import SuperPointMotionNode
from nexus_vision_localization.target_metric_node import TargetMetricNode
from nexus_fusion_localization.target_kinematic_node import TargetKinematicFusionNode
from nexus_viz_dashboard.localization_dashboard_node import LocalizationDashboardNode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--detector', required=True)
    parser.add_argument('--superpoint', required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '89':
        raise ValueError('Use isolated ROS_DOMAIN_ID=89 for synthetic inputs')
    args.output.mkdir(parents=True, exist_ok=False)
    calibration = args.output / 'synthetic_geometry.yaml'
    calibration.write_text(yaml.safe_dump({
        'schema_version': 1, 'calibration_id': 'synthetic_gpu_chain',
        'calibration_status': 'synthetic_test', 'camera_frame': 'camera_optical_frame',
        'image_rectified': True, 'transform_body_camera': np.eye(4).tolist(),
        'extrinsic_covariance': np.zeros((6, 6)).tolist(), 'pixel_sigma_px': 1.}))
    rclpy.init(args=['--ros-args', '-p', f'calibration_file:={calibration}',
        '-p', 'allow_test_calibration:=true', '-p', 'enable_target_tracking:=true',
        '-p', f'nexus_object_detection:model_manifest:={args.detector}',
        '-p', f'nexus_superpoint_motion:model_manifest:={args.superpoint}',
        '-p', 'input_mode:=SIMULATION', '-p', 'run_id:=synthetic_gpu_chain'])
    nodes = [ObjectDetectionNode(), SuperPointMotionNode(), TargetMetricNode(),
             TargetKinematicFusionNode(), LocalizationDashboardNode(), Node('reference_coordinate_fixture')]
    detector, frontend, metric, fusion, dashboard, driver = nodes
    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    bridge = CvBridge()
    pubs = {name: driver.create_publisher(kind, topic, 20) for name, kind, topic in [
        ('image', Image, '/camera/image_rect'), ('camera', CameraInfo, '/camera/camera_info'),
        ('pose', Odometry, '/nexus/platform/odom'), ('reference', Image, '/nexus/vision/target_reference_image'),
        ('request', String, '/nexus/vision/target_requests')]}
    requests, tracks, geometries, fused, snapshots, statuses = [], [], [], [], [], []
    for topic, dest in [('/nexus/vision/target_request_status', requests),
                        ('/nexus/vision/target_tracks', tracks),
                        ('/nexus/viz/localization_state', snapshots),
                        ('/nexus/vision/target_geometry_status', statuses)]:
        driver.create_subscription(String, topic, lambda msg, dest=dest: dest.append(json.loads(msg.data)), 20)
    driver.create_subscription(TargetKinematicState, '/nexus/vision/target_kinematics', geometries.append, 20)
    driver.create_subscription(TargetKinematicState, '/nexus/target/kinematics', fused.append, 20)

    def spin(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.001)

    def image_message(pixels, stamp):
        msg = bridge.cv2_to_imgmsg(pixels, 'bgr8')
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
        msg.header.frame_id = 'camera_optical_frame'
        return msg

    result = {'synthetic_inputs': True, 'real_model_inference': True, 'domain': 89}
    try:
        texture = np.random.default_rng(45).integers(0, 256, (480, 640, 3), dtype=np.uint8)
        texture = cv2.GaussianBlur(texture, (5, 5), 0)
        for i in range(12):
            cv2.putText(texture, str(i), (190 + (i % 4) * 60, 165 + (i // 4) * 65),
                        cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)
        frontend.model.infer(texture)
        detector.model.infer(texture)
        spin(1.)
        stamp = driver.get_clock().now().nanoseconds
        frame = image_message(texture, stamp)
        camera = CameraInfo()
        camera.header = frame.header
        camera.width, camera.height = 640, 480
        camera.k = [500., 0., 320., 0., 500., 240., 0., 0., 1.]
        camera.p = [500., 0., 320., 0., 0., 500., 240., 0., 0., 0., 1., 0.]
        pubs['camera'].publish(camera)
        pubs['reference'].publish(frame)
        pubs['request'].publish(String(data=json.dumps({
            'schema_version': 1, 'request_id': 'gpu_chain_selection', 'target_id': 'selected_patch',
            'sample_timestamp_ns': stamp, 'frame_id': 'camera_optical_frame',
            'bbox_xywh_px': [180, 120, 280, 240], 'motion_model': 'static'})))
        spin(1.)
        assert any(item['state'] == 'registered' for item in requests), requests
        print('REGISTERED via real SuperPoint reference inference', flush=True)
        for i in range(24):
            center_x = .015 * i
            pixels = cv2.warpAffine(texture, np.float32([[1, 0, -500 * center_x / 3], [0, 1, 0]]),
                                    (640, 480), borderMode=cv2.BORDER_REFLECT)
            stamp = driver.get_clock().now().nanoseconds
            frame = image_message(pixels, stamp)
            pose = Odometry()
            pose.header.stamp = frame.header.stamp
            pose.header.frame_id, pose.child_frame_id = 'map', 'base_link'
            pose.pose.pose.position.x = center_x
            pose.pose.pose.orientation.w = 1.
            pubs['pose'].publish(pose)
            pubs['image'].publish(frame)
            spin(.12)
        spin(.1)
        observed = [m for m in fused if m.valid and m.target_id == 'selected_patch']
        displayed = [t for s in snapshots for t in s['targets']
                     if t['target_id'] == 'selected_patch' and t['display_state'] == 'observed']
        result.update(registration=requests, track_packets=len(tracks),
                      valid_geometry=sum(m.valid for m in geometries), valid_fused=len(observed),
                      displayed_samples=len(displayed), geometry_status=statuses[-8:])
        if observed:
            m = observed[-1]
            result['last_position_m'] = [m.pose.position.x, m.pose.position.y, m.pose.position.z]
            result['depth_error_m'] = abs(m.pose.position.z - 3.)
        assert observed and displayed, result
        assert result['depth_error_m'] < .1, result
        result['passed'] = True
    finally:
        (args.output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, indent=2), flush=True)


if __name__ == '__main__':
    main()
