"""DLO spline fitting node.

Subscribes to raw OptiTrack unlabeled markers, sorts them into a consistent
wire path, fits a B-spline, resamples to exactly num_kp equidistant keypoints,
and publishes the result as a PointsArray.
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from my_msgs.msg import PointsArray

from simplepredict.dataset_utils.spline_utils import fit_spline_unordered

from njc_ros.conversions import marker_points_to_numpy


class DLOSplineNode(Node):
    def __init__(self):
        super().__init__('dlo_spline_node')

        # Parameters
        self.declare_parameter('num_kp', 14)
        self.declare_parameter('momentum_weight', 0.2)
        self.declare_parameter('spline_k', 3)
        self.declare_parameter('min_markers', 10)
        self.declare_parameter('rate', 30.0)

        self.num_kp = self.get_parameter('num_kp').get_parameter_value().integer_value
        self.momentum_weight = self.get_parameter('momentum_weight').get_parameter_value().double_value
        self.spline_k = self.get_parameter('spline_k').get_parameter_value().integer_value
        self.min_markers = self.get_parameter('min_markers').get_parameter_value().integer_value
        rate = self.get_parameter('rate').get_parameter_value().double_value

        # Latest marker data
        self._latest_marker_msg: Marker | None = None

        # Subscriber
        self.create_subscription(
            Marker,
            '/natnet/unlabeled_marker_data',
            self._marker_cb,
            10,
        )

        # Publisher
        self.pub = self.create_publisher(PointsArray, '/njc/cable_keypoints', 10)

        # Timer
        self.create_timer(1.0 / rate, self._timer_cb)

        self.get_logger().info(
            f'dlo_spline_node started: num_kp={self.num_kp}, '
            f'min_markers={self.min_markers}, rate={rate} Hz'
        )

    def _marker_cb(self, msg: Marker):
        self._latest_marker_msg = msg

    def _timer_cb(self):
        if self._latest_marker_msg is None:
            return

        msg = self._latest_marker_msg

        # Staleness check: skip if marker data age > 100ms
        now = self.get_clock().now()
        msg_time = Time.from_msg(msg.header.stamp)
        age_sec = (now - msg_time).nanoseconds * 1e-9
        if age_sec > 0.1:
            return

        # Extract points
        points = marker_points_to_numpy(msg)
        if len(points) < self.min_markers:
            return

        # Sort + spline fit + resample
        try:
            resampled, _ = fit_spline_unordered(
                points,
                num_samples=self.num_kp,
                k=self.spline_k,
                use_gpu=False,
            )
        except Exception as e:
            self.get_logger().warn(f'Spline fitting failed: {e}')
            return

        # Publish PointsArray
        out = PointsArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id
        out.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in resampled]
        self.pub.publish(out)


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
