"""NJC controller node — wraps the NJC inference pipeline."""

import rclpy
from rclpy.node import Node


class NJCControllerNode(Node):
    def __init__(self):
        super().__init__('njc_controller_node')
        self.get_logger().info('njc_controller_node placeholder — not yet implemented')


def main(args=None):
    rclpy.init(args=args)
    node = NJCControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
