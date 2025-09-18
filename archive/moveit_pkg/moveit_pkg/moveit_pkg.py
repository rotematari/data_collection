#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MoveItNode(Node):
    def __init__(self):
        super().__init__('moveit_node')
        # Example parameter and timer
        self.declare_parameter('rate', 1.0)
        rate = self.get_parameter('rate').get_parameter_value().double_value
        period = 1.0 / rate if rate > 0.0 else 1.0
        self.timer = self.create_timer(period, self._on_timer)
        self.get_logger().info('moveit_node started')

    def _on_timer(self):
        # Periodic work placeholder
        self.get_logger().debug('moveit_node heartbeat')


def main(args=None):
    rclpy.init(args=args)
    node = MoveItNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
