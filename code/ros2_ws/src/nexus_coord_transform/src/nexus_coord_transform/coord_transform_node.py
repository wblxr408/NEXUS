#!/usr/bin/env python3
import math
import pathlib

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

from nexus_msgs.msg import TargetObservation

from nexus_coord_transform.geometry import quaternion_to_matrix, rotate_position_covariance


class CoordTransformNode(Node):
    def __init__(self):
        super().__init__("nexus_coord_transform")
        self.declare_parameter("transform_file", "")
        self.declare_parameter("target_frame", "map")
        # Subscription callbacks must not wait for TF; unavailable transforms
        # are represented by an explicit INVALID message instead.
        self.declare_parameter("transform_timeout_ms", 0.0)
        self.declare_parameter("vision_input_topic", "/nexus/vision/target_observation")
        self.declare_parameter("vision_output_topic", "/nexus/vision/map_target_observation")
        self.declare_parameter("uwb_input_topic", "/nexus/uwb/raw_target_observation")
        self.declare_parameter("uwb_output_topic", "/nexus/uwb/target_observation")
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._timeout = Duration(
            nanoseconds=int(float(self.get_parameter("transform_timeout_ms").value) * 1e6))
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._broadcaster = StaticTransformBroadcaster(self)
        self._vision_publisher = self.create_publisher(
            TargetObservation, str(self.get_parameter("vision_output_topic").value), 10)
        self._uwb_publisher = self.create_publisher(
            TargetObservation, str(self.get_parameter("uwb_output_topic").value), 10)
        self.create_subscription(
            TargetObservation, str(self.get_parameter("vision_input_topic").value),
            lambda message: self._observation_callback(message, self._vision_publisher), 10)
        self.create_subscription(
            TargetObservation, str(self.get_parameter("uwb_input_topic").value),
            lambda message: self._observation_callback(message, self._uwb_publisher), 10)
        transform_file = str(self.get_parameter("transform_file").value)
        if not transform_file:
            self.get_logger().warning(
                "transform_file is empty; waiting for externally published verified transforms")
            return
        self._publish_from_file(pathlib.Path(transform_file))

    def _publish_from_file(self, path):
        if not path.is_file():
            self.get_logger().error(f"transform file does not exist: {path}")
            return
        with path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream) or {}
        transforms = document.get("transforms")
        if not isinstance(transforms, list):
            self.get_logger().error("transform file must contain a transforms list")
            return
        if not transforms:
            self.get_logger().warning(
                "transform file has no measured transforms; map output remains invalid until hardware calibration")
            return
        messages = []
        for item in transforms:
            try:
                parent = str(item["parent"])
                child = str(item["child"])
                translation = [float(value) for value in item["translation"]]
                rotation = [float(value) for value in item["rotation_xyzw"]]
                if len(translation) != 3 or len(rotation) != 4:
                    raise ValueError("translation must have 3 and rotation_xyzw 4 values")
                if not all(math.isfinite(value) for value in translation + rotation):
                    raise ValueError("transform values must be finite")
                norm = math.sqrt(sum(value * value for value in rotation))
                if norm <= 0 or not math.isclose(norm, 1.0, abs_tol=1e-6):
                    raise ValueError("rotation_xyzw must be a unit quaternion")
            except (KeyError, TypeError, ValueError) as error:
                self.get_logger().error(f"invalid transform entry: {error}")
                return
            message = TransformStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = parent
            message.child_frame_id = child
            message.transform.translation.x = translation[0]
            message.transform.translation.y = translation[1]
            message.transform.translation.z = translation[2]
            message.transform.rotation.x = rotation[0]
            message.transform.rotation.y = rotation[1]
            message.transform.rotation.z = rotation[2]
            message.transform.rotation.w = rotation[3]
            messages.append(message)
        self._broadcaster.sendTransform(messages)

    @staticmethod
    def _stamp_ns(message):
        return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)

    def _invalid_output(self, message, reason):
        output = TargetObservation()
        output.header = message.header
        output.header.frame_id = self._target_frame
        sample_ns = self._stamp_ns(message)
        output.receive_timestamp_ns = max(self.get_clock().now().nanoseconds, sample_ns)
        output.target_id = message.target_id
        output.source_mode = message.source_mode
        output.validity = TargetObservation.VALIDITY_INVALID
        output.invalid_reason = str(reason)
        output.last_valid_sample_timestamp_ns = message.last_valid_sample_timestamp_ns
        output.unit = "m"
        output.pose.position.x = float("nan")
        output.pose.position.y = float("nan")
        output.pose.position.z = float("nan")
        output.pose.orientation.x = float("nan")
        output.pose.orientation.y = float("nan")
        output.pose.orientation.z = float("nan")
        output.pose.orientation.w = float("nan")
        output.covariance = [float("nan")] * 36
        output.confidence = 0.0
        return output

    def _observation_callback(self, message, publisher):
        if message.validity != TargetObservation.VALIDITY_VALID:
            publisher.publish(self._invalid_output(
                message, message.invalid_reason or "upstream_invalid"))
            return
        if message.unit != "m":
            publisher.publish(self._invalid_output(message, "invalid_unit"))
            return
        if not message.header.frame_id:
            publisher.publish(self._invalid_output(message, "invalid_frame"))
            return
        try:
            transform = self._buffer.lookup_transform(
                self._target_frame, message.header.frame_id,
                Time.from_msg(message.header.stamp), timeout=self._timeout)
            transformed_pose = do_transform_pose(message.pose, transform)
            rotation = quaternion_to_matrix([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ])
            covariance = np.asarray(message.covariance, dtype=float).reshape(6, 6)
            rotated_position_covariance = rotate_position_covariance(
                covariance[:3, :3], rotation)
        except (TransformException, TypeError, ValueError) as error:
            self.get_logger().warning(
                f"cannot transform target={message.target_id} from "
                f"{message.header.frame_id} to {self._target_frame}: {error}")
            publisher.publish(self._invalid_output(message, "transform_unavailable"))
            return
        output = TargetObservation()
        output.header = message.header
        output.header.frame_id = self._target_frame
        output.receive_timestamp_ns = max(
            self.get_clock().now().nanoseconds, message.receive_timestamp_ns,
            self._stamp_ns(message))
        output.target_id = message.target_id
        output.source_mode = message.source_mode
        output.validity = TargetObservation.VALIDITY_VALID
        output.invalid_reason = ""
        output.last_valid_sample_timestamp_ns = self._stamp_ns(message)
        output.unit = "m"
        output.pose = transformed_pose
        output_covariance = np.zeros((6, 6), dtype=float)
        output_covariance[:3, :3] = rotated_position_covariance
        output.covariance = output_covariance.reshape(-1).tolist()
        output.confidence = message.confidence
        publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = CoordTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
