"""DLO spline fitting node — sorts, fits, and resamples OptiTrack markers."""

import rclpy
from rclpy.node import Node


class DLOSplineNode(Node):
    def __init__(self):
        super().__init__('dlo_spline_node')
        self.get_logger().info('dlo_spline_node placeholder — not yet implemented')


def main(args=None):
    rclpy.init(args=args)
    node = DLOSplineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
