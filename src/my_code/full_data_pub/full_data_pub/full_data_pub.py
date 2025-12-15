#!/usr/bin/env python3
import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from rclpy.time import Time
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState, Image
from visualization_msgs.msg import Marker

# tf imports
from rclpy.duration import Duration
# from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros import Buffer, TransformListener 
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf_transformations 


class FullDataPub(Node):
    def __init__(self):
        super().__init__("full_data_pub")
        
        
        
        self.declare_parameter("rate", 10)
        self.declare_parameter("num_markers", 12)
        self.declare_parameter("base_frame", "base_link")
        # t base_world
        self.declare_parameter("t_calib", [0.0, 0.0, 0.0])
        self.declare_parameter("q_calib", [0.0, 0.0, 0.0, 1.0])  # x,y,z,w

        self.rate = self.get_parameter("rate").get_parameter_value().integer_value
        self.num_markers = self.get_parameter("num_markers").get_parameter_value().integer_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.t_calib = self.get_parameter("t_calib").get_parameter_value().double_array_value
        self.q_calib = self.get_parameter("q_calib").get_parameter_value().double_array_value
        # log the rate in green
        self.get_logger().info(f"\033[92mFullDataPub node started with rate: {self.rate} Hz\033[0m")
        # QoS for typical sensor streams (images, mocap, etc.)
        self.image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # QoS for ref images: TRANSIENT_LOCAL so they are latched.
        self.ref_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Storage for latest messages
        self.bad_readings_count = 0
        self.latest = {
            "unlabeled_markers":  None,
            "joint_states": None,
            "digit_D21118_image": None,
            "digit_D21122_image": None,
            "digit_D21123_image": None,
            "digit_D21124_image": None,
        }

        # Which keys we require before publishing a "full_data" sample
        self.required_keys = [
            "unlabeled_markers",
            "joint_states",
            "digit_D21118_image",
            "digit_D21122_image",
            "digit_D21123_image",
            "digit_D21124_image",
        ]

        # Keep track of whether we already published refs
        self.ref_published = {
            "D21118": False,
            "D21122": False,
            "D21123": False,
            "D21124": False,
        }

        
        # TF buffer and listener
        self.buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.listener = TransformListener(self.buffer, self)
        
        # ---------- Subscriptions ----------
        self.set_up_subscriptions()


        # ---------- Publishers ----------
        self.set_up_publishers()
        
        
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
        self.get_logger().info(
            "FullDataPub node started."
        )


    def set_up_subscriptions(self):

        # /natnet/unlabeled_marker_data  (replace DummyUnlabeledMarkerData)
        self.sub_unlabeled_markers = self.create_subscription(
            Marker,
            "/natnet/unlabeled_marker_data",
            self.cb_unlabeled_markers,
            self.image_qos,
        )


        # /joint_state (note: often called /joint_states in many setups)
        self.sub_joint_state = self.create_subscription(
            JointState,
            "/joint_states",
            self.cb_joint_states,
            self.image_qos,
        )

        # DIGIT images (image_raw)
        self.sub_digit_D21118_img = self.create_subscription(
            Image,
            "/digit/D21118/image_raw",
            self.cb_digit_D21118_image,  # MASTER
            self.image_qos,
        )
        self.sub_digit_D21122_img = self.create_subscription(
            Image,
            "/digit/D21122/image_raw",
            self.cb_digit_D21122_image,
            self.image_qos,
        )
        self.sub_digit_D21123_img = self.create_subscription(
            Image,
            "/digit/D21123/image_raw",
            self.cb_digit_D21123_image,
            self.image_qos,
        )
        self.sub_digit_D21124_img = self.create_subscription(
            Image,
            "/digit/D21124/image_raw",
            self.cb_digit_D21124_image,
            self.image_qos,
        )

        # DIGIT ref images – publish only once, using self.ref_qos
        self.sub_digit_D21118_ref = self.create_subscription(
            Image,
            "/digit/D21118/ref_image",
            self.cb_digit_ref_factory("D21118"),
            self.ref_qos,
        )
        self.sub_digit_D21122_ref = self.create_subscription(
            Image,
            "/digit/D21122/ref_image",
            self.cb_digit_ref_factory("D21122"),
            self.ref_qos,
        )
        self.sub_digit_D21123_ref = self.create_subscription(
            Image,
            "/digit/D21123/ref_image",
            self.cb_digit_ref_factory("D21123"),
            self.ref_qos,
        )
        self.sub_digit_D21124_ref = self.create_subscription(
            Image,
            "/digit/D21124/ref_image",
            self.cb_digit_ref_factory("D21124"),
            self.ref_qos,
        )
    def set_up_publishers(self):
        # /full_data/natnet_fixed_pose
        self.pub_natnet_fixed_pose = self.create_publisher(
            PoseStamped,
            "/full_data/natnet_fixed_pose",
            self.pose_qos,
        )
        # /full_data/natnet_robot_ee_pose
        self.pub_natnet_robot_ee_pose = self.create_publisher(
            PoseStamped,
            "/full_data/natnet_robot_ee_pose",
            self.pose_qos,
        )

        # /full_data/natnet/unlabeled_marker_data
        self.pub_unlabeled_markers = self.create_publisher(
            Marker,
            "/full_data/natnet/unlabeled_marker_data",
            self.pose_qos,
        )


        # /full_data/joint_states
        self.pub_joint_states = self.create_publisher(
            JointState,
            "/full_data/joint_states",
            self.image_qos,
        )

        # /full_data/digit/.../image_raw
        self.pub_digit_D21118_img = self.create_publisher(
            Image,
            "/full_data/digit/D21118/image_raw",
            self.image_qos,
        )
        self.pub_digit_D21122_img = self.create_publisher(
            Image,
            "/full_data/digit/D21122/image_raw",
            self.image_qos,
        )
        self.pub_digit_D21123_img = self.create_publisher(
            Image,
            "/full_data/digit/D21123/image_raw",
            self.image_qos,
        )
        self.pub_digit_D21124_img = self.create_publisher(
            Image,
            "/full_data/digit/D21124/image_raw",
            self.image_qos,
        )

        # /full_data/digit/.../ref_image  (latched: TRANSIENT_LOCAL)
        self.pub_digit_D21118_ref = self.create_publisher(
            Image,
            "/full_data/digit/D21118/ref_image",
            self.ref_qos,
        )
        self.pub_digit_D21122_ref = self.create_publisher(
            Image,
            "/full_data/digit/D21122/ref_image",
            self.ref_qos,
        )
        self.pub_digit_D21123_ref = self.create_publisher(
            Image,
            "/full_data/digit/D21123/ref_image",
            self.ref_qos,
        )
        self.pub_digit_D21124_ref = self.create_publisher(
            Image,
            "/full_data/digit/D21124/ref_image",
            self.ref_qos,
        )
    # ---------- Callbacks: store latest messages ----------


    def cb_unlabeled_markers(self, msg):
        if msg is None:
            return

        points = msg.points
        count = len(points)

        if count != self.num_markers:
            self.get_logger().warn(
                f"Expected {self.num_markers} markers, but got {count}."
            )
            self.bad_readings_count += 1
            if self.bad_readings_count >= 50:
                self.get_logger().error(
                    f"Received {self.bad_readings_count} consecutive bad readings on unlabeled_markers. Resetting stored data."
                )
                self.latest["unlabeled_markers"] = None
        else:
            self.latest["unlabeled_markers"] = msg
            self.bad_readings_count = 0

    def cb_joint_states(self, msg: JointState):
        if msg is not None:
            self.latest["joint_states"] = msg

    def cb_digit_D21118_image(self, msg: Image):
        self.latest["digit_D21118_image"] = msg


    def cb_digit_D21122_image(self, msg: Image):
        self.latest["digit_D21122_image"] = msg

    def cb_digit_D21123_image(self, msg: Image):
        self.latest["digit_D21123_image"] = msg

    def cb_digit_D21124_image(self, msg: Image):
        self.latest["digit_D21124_image"] = msg

    # ---------- Ref image handling: publish only once ----------

    def cb_digit_ref_factory(self, serial: str):
        """
        Returns a callback that handles one serial's ref_image topic.
        Only publishes once on /full_data/digit/<serial>/ref_image.
        """
        def cb(msg: Image):
            if self.ref_published[serial]:
                return

            # Pick the right publisher
            if serial == "D21118":
                pub = self.pub_digit_D21118_ref
            elif serial == "D21122":
                pub = self.pub_digit_D21122_ref
            elif serial == "D21123":
                pub = self.pub_digit_D21123_ref
            elif serial == "D21124":
                pub = self.pub_digit_D21124_ref
            else:
                self.get_logger().warn(f"Unknown serial for ref_image: {serial}")
                return

            pub.publish(msg)
            self.ref_published[serial] = True
            self.get_logger().info(f"Published ref_image once for {serial} under /full_data")
        return cb

    # ---------- Helper: check if we have all required ----------

    def is_bundle_synchronized(self, master_stamp: Time, threshold_sec: float) -> bool:
        """
        Checks if all required secondary sensors are within `threshold_sec` 
        of the master timestamp.
        """
        master_ns = master_stamp.nanoseconds
        
        for key in self.required_keys:
            msg = self.latest[key]
            
            # 1. Missing Data check
            if msg is None:
                # Allow a warm-up warning, but don't crash
                self.get_logger().info(f"Missing key: {key}")
                return False

            # 2. Synchrony Check
            # We compare the msg timestamp to the MASTER timestamp, not wall-clock.
            msg_time_ns = Time.from_msg(msg.header.stamp).nanoseconds
            delta_sec = abs(master_ns - msg_time_ns) / 1e9

            if delta_sec > threshold_sec:
                self.get_logger().warn(
                    f"Desync detected: {key} is {delta_sec:.4f}s apart from master."
                )
                return False
        
        return True
    
    def timer_callback(self):
        # 1. Acquire Master Data
        master_msg = self.latest["digit_D21118_image"] # The Master
        
        if master_msg is None:
            self.get_logger().info("Master DIGIT D21118 image is None.")
            return 

        # 2. Extract Master Time
        # This effectively becomes the "capture time" for the whole bundle
        master_stamp = Time.from_msg(master_msg.header.stamp)

        # 3. Check Synchrony
        # Threshold: e.g., 0.1s. If sensors drift more than 10ms apart, discard.
        if not self.is_bundle_synchronized(master_stamp, threshold_sec=0.02):
            return

        # 4. Publish Bundle using Master Time
        self.publish_full_data(master_stamp)
    def get_transform_robust(self, target, source, stamp, timeout_sec=0.1):
        """
        Attempts to get the transform at exactly 'stamp'.
        If that requires extrapolation into the future (msg is newer than TF),
        it falls back to the LATEST available transform.
        """
        # 1. Try Strict Lookup
        try:
            return self.buffer.lookup_transform(
                target, 
                source, 
                stamp, 
                timeout=Duration(seconds=timeout_sec)
            )
        except Exception:
            # 2. Fallback: Get the Very Latest
            # The image is slightly ahead of the mocap. Just grab the newest mocap pose.
            try:
                latest_tf = self.buffer.lookup_transform(
                    target, 
                    source, 
                    Time(), # "0" means latest
                    timeout=Duration(seconds=0.0) # Instant check
                )
                
                # Optional: Check if the "latest" is reasonably close (e.g. within 50ms)
                # to avoid using stale data if mocap died.
                latest_stamp_ns = Time.from_msg(latest_tf.header.stamp).nanoseconds
                req_stamp_ns = stamp.nanoseconds
                delta_sec = abs(latest_stamp_ns - req_stamp_ns) / 1e9
                
                if delta_sec < 0.05: # 50ms tolerance
                    return latest_tf
                else:
                    self.get_logger().warn(f"Fallback TF too old: {delta_sec:.4f}s diff")
                    return None
                    
            except Exception:
                return None

    def publish_full_data(self, stamp: Time):
        """
        Publish data. 
        CRITICAL: We look up TF at `stamp` (master time), not `Time()` (latest).
        """
        stamp_msg = stamp.to_msg()
        # --- TF Lookup ---
        # try:
        #     # We increase timeout to 0.05 (50ms) to allow TF buffer to catch up 
        #     # if the image arrived slightly ahead of the joint state processing.
        #     lookup_timeout = Duration(seconds=0.01)
            
        #     ee_tf = self.buffer.lookup_transform(
        #         target_frame="base_link",
        #         source_frame="natnet_robot_ee",
        #         time=stamp, # strict time lookup
        #         timeout=lookup_timeout
        #     )
        #     fixed_tf = self.buffer.lookup_transform(
        #         target_frame="base_link",
        #         source_frame="natnet_fixed_ee",
        #         time=stamp, # strict time lookup
        #         timeout=lookup_timeout
        #     )

        # except Exception as e:
        #     # Common in the first few seconds or if packet loss occurs
        #     self.get_logger().warn(f"TF Lookup failed for time {stamp.nanoseconds}: {e}")
            
        #     return
        ee_tf = self.get_transform_robust(
            target="base_link",
            source="end_effector_link",
            stamp=stamp,
            timeout_sec=0.01,
        )
        fixed_tf = self.get_transform_robust(
            target="base_link",
            source="natnet_fixed_ee",
            stamp=stamp,
            timeout_sec=0.01,
        )
        # --- Helper for Rotation correction (+X -> +Z) ---
        def apply_rotation_offset(tf_stamped):
            q_tf = tf_stamped.transform.rotation
            q_tf_array = [q_tf.x, q_tf.y, q_tf.z, q_tf.w]
            q_offset = tf_transformations.quaternion_from_euler(0, -1.57079632679, 0)
            return tf_transformations.quaternion_multiply(q_tf_array, q_offset)

        # --- Publish Fixed Pose to robot base_link ---
        q_fixed_res = apply_rotation_offset(fixed_tf)
        
        pub_fixed_pose_msg = PoseStamped()
        pub_fixed_pose_msg.header.stamp = stamp_msg # Use Master Stamp
        pub_fixed_pose_msg.header.frame_id = fixed_tf.header.frame_id
        pub_fixed_pose_msg.pose.position.x = fixed_tf.transform.translation.x
        pub_fixed_pose_msg.pose.position.y = fixed_tf.transform.translation.y
        pub_fixed_pose_msg.pose.position.z = fixed_tf.transform.translation.z
        pub_fixed_pose_msg.pose.orientation.x = q_fixed_res[0]
        pub_fixed_pose_msg.pose.orientation.y = q_fixed_res[1]
        pub_fixed_pose_msg.pose.orientation.z = q_fixed_res[2]
        pub_fixed_pose_msg.pose.orientation.w = q_fixed_res[3]
        self.pub_natnet_fixed_pose.publish(pub_fixed_pose_msg)

        # --- Publish EE Pose  relative to robot base_link ---
        q_ee_res = apply_rotation_offset(ee_tf)
        
        pub_ee_pose_msg = PoseStamped()
        pub_ee_pose_msg.header.stamp = stamp_msg # Use Master Stamp
        pub_ee_pose_msg.header.frame_id = ee_tf.header.frame_id
        pub_ee_pose_msg.pose.position.x = ee_tf.transform.translation.x
        pub_ee_pose_msg.pose.position.y = ee_tf.transform.translation.y
        pub_ee_pose_msg.pose.position.z = ee_tf.transform.translation.z
        pub_ee_pose_msg.pose.orientation.x = q_ee_res[0]
        pub_ee_pose_msg.pose.orientation.y = q_ee_res[1]
        pub_ee_pose_msg.pose.orientation.z = q_ee_res[2]
        pub_ee_pose_msg.pose.orientation.w = q_ee_res[3]
        self.pub_natnet_robot_ee_pose.publish(pub_ee_pose_msg)

        # --- Publish Sensors relative to robot base ---
        # 1. Unlabeled Markers in base frame
        msg = copy.deepcopy(self.latest["unlabeled_markers"])
        R_natnet_world = tf_transformations.quaternion_matrix([
            self.q_calib[0], self.q_calib[1], self.q_calib[2], self.q_calib[3]
        ])
        T_natnet_world = tf_transformations.translation_matrix([
            self.t_calib[0], self.t_calib[1], self.t_calib[2]
        ])
        # self.get_logger().info(f"T_natnet_world:\n{T_natnet_world}")
        # Create the combined 4x4 Homogeneous Transformation Matrix
        M_transform = tf_transformations.concatenate_matrices(T_natnet_world, R_natnet_world)
        # self.get_logger().info(f"M_transform before inv:\n{M_transform}")

        msg.header.frame_id = self.base_frame
        for point in msg.points:
            
            point_np_natnetw = np.array([point.x, point.y, point.z, 1.0])  # Homogeneous coords
            point_np_base = M_transform.dot(point_np_natnetw)
            # self.get_logger().info(f"Transformed point: {point_np_base}")
            point.x = float(point_np_base[0])
            point.y = float(point_np_base[1])
            point.z = float(point_np_base[2])
            
            
        msg.header.stamp = stamp_msg
        self.pub_unlabeled_markers.publish(msg)

        # 2. Joint States
        msg = copy.deepcopy(self.latest["joint_states"])
        msg.header.stamp = stamp_msg
        self.pub_joint_states.publish(msg)

        # 3. Images
        # We publish the images with the MASTER stamp. 
        # Note: D21118 (the master) will naturally match. Others are forced to match.
        for _, key, pub in [
            ("D21118", "digit_D21118_image", self.pub_digit_D21118_img),
            ("D21122", "digit_D21122_image", self.pub_digit_D21122_img),
            ("D21123", "digit_D21123_image", self.pub_digit_D21123_img),
            ("D21124", "digit_D21124_image", self.pub_digit_D21124_img),
        ]:
            img = self.latest.get(key)
            img_copy = copy.deepcopy(img)
            img_copy.header.stamp = stamp_msg
            pub.publish(img_copy)

        self.get_logger().debug("Published synchronized full_data sample.")


def main(args=None):
    rclpy.init(args=args)
    node = FullDataPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
