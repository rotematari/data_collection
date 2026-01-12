#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped

class EndEffectorPosePublisher(Node):
    def __init__(self):
        super().__init__('end_effector_pose_publisher')
        # Declare configurable parameters
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('end_effector_frame', 'end_effector_link')
        self.declare_parameter('publish_topic', 'end_effector_pose')
        self.declare_parameter('publish_rate', 500.0)  # Hz
        self.declare_parameter('timeout_sec', 0.0)

        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.ee_frame = self.get_parameter('end_effector_frame').get_parameter_value().string_value
        topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.timeout_sec = self.get_parameter('timeout_sec').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(PoseStamped, topic, 10)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        timer_period = 1.0 / rate if rate > 0.0 else 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f"Publishing {self.ee_frame} pose in {self.base_frame} on '{topic}' at {rate} Hz")
    
    
    def get_transform_robust(self, target, source)-> TransformStamped | None:
        """
        Attempts to get the transform at exactly 'stamp'.
        If that requires extrapolation into the future (msg is newer than TF),
        it falls back to the LATEST available transform.
        """
        # 1. Try Strict Lookup
        try:
            ee_to_base = self.tf_buffer.lookup_transform(
                target, 
                source, 
                Time(), # "0" means latest
                timeout=Duration(seconds=self.timeout_sec)  # Use configured timeout
            )
            return ee_to_base
        except Exception:
            return None
            
    def timer_callback(self):
        
        # self.get_logger().info(f"Received pose at time {time.sec}.{time.nanosec} for {self.ee_frame} in {self.base_frame}")
        try:
            transform = self.get_transform_robust(
                target=self.base_frame,
                source=self.ee_frame,
            )
            if transform is None:
                self.get_logger().warn("Could not get transform for publishing end effector pose.")
                return
        except Exception as ex:  # Fallback if specific tf2 exceptions not exposed in this environment
            self.get_logger().warn(f"TF lookup failed: {ex}", throttle_duration_sec=5.0)
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.pose.position.x = transform.transform.translation.x
        msg.pose.position.y = transform.transform.translation.y
        msg.pose.position.z = transform.transform.translation.z
        msg.pose.orientation = transform.transform.rotation

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
