#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class PreflightNode(Node):
    """Report readiness without manufacturing a sensor input."""

    def __init__(self):
        super().__init__("nexus_bringup_preflight")
        self.declare_parameter("hardware_required", True)
        self._reported = False
        self.create_timer(1.0, self._report)

    def _report(self):
        if self._reported:
            return
        self._reported = True
        if bool(self.get_parameter("hardware_required").value):
            self.get_logger().warning(
                "hardware_required=true; waiting for ROS1 fcu_core and real sensor inputs; "
                "no synthetic source is started")
        else:
            self.get_logger().info("software-only preflight; no sensor source is started")


def main(args=None):
    rclpy.init(args=args)
    node = PreflightNode()
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
