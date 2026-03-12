"""ROS <-> NumPy conversion utilities for NJC."""

import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from my_msgs.msg import PointsArray
from visualization_msgs.msg import Marker


def pose_stamped_to_pos_quat(msg: PoseStamped) -> tuple[np.ndarray, np.ndarray]:
    """Extract position (3,) and quaternion xyzw (4,) from a PoseStamped."""
    p = msg.pose.position
    q = msg.pose.orientation
    pos = np.array([p.x, p.y, p.z], dtype=np.float32)
    quat = np.array([q.x, q.y, q.z, q.w], dtype=np.float32)
    return pos, quat


def cmd_to_twist_stamped(cmd, frame_id: str, stamp) -> TwistStamped:
    """Convert an NJC CMDMsg to a ROS TwistStamped.

    Args:
        cmd: njc.inference.CMDMsg with .linear (3,) and .angular (3,).
        frame_id: TF frame for the header.
        stamp: rclpy Time to use as the header stamp.
    """
    msg = TwistStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.twist.linear.x = float(cmd.linear[0])
    msg.twist.linear.y = float(cmd.linear[1])
    msg.twist.linear.z = float(cmd.linear[2])
    msg.twist.angular.x = float(cmd.angular[0])
    msg.twist.angular.y = float(cmd.angular[1])
    msg.twist.angular.z = float(cmd.angular[2])
    return msg


def marker_points_to_numpy(msg: Marker) -> np.ndarray:
    """Extract (N, 3) float32 array from a Marker POINTS message."""
    pts = msg.points
    return np.array([[p.x, p.y, p.z] for p in pts], dtype=np.float32)


def points_array_to_numpy(msg: PointsArray) -> np.ndarray:
    """Extract (N, 3) float32 array from a PointsArray message."""
    pts = msg.points
    return np.array([[p.x, p.y, p.z] for p in pts], dtype=np.float32)
