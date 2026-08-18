#!/usr/bin/env python3
import math
import pathlib

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class CoordTransformNode(Node):
    def __init__(self):
        super().__init__("nexus_coord_transform")
        self.declare_parameter("transform_file", "")
        self._broadcaster = StaticTransformBroadcaster(self)
        transform_file = str(self.get_parameter("transform_file").value)
        if not transform_file:
            self.get_logger().error("transform_file is required; real frame geometry must not be guessed")
            return
        self._publish_from_file(pathlib.Path(transform_file))

    def _publish_from_file(self, path):
        if not path.is_file():
            self.get_logger().error(f"transform file does not exist: {path}")
            return
        with path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream) or {}
        transforms = document.get("transforms")
        if not isinstance(transforms, list) or not transforms:
            self.get_logger().error("transform file must contain a non-empty transforms list")
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


def main(args=None):
    rclpy.init(args=args)
    node = CoordTransformNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
