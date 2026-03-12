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
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState, Image
from visualization_msgs.msg import Marker

# tf imports
from rclpy.duration import Duration
# from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros import Buffer, TransformListener 
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
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # QoS for ref images: TRANSIENT_LOCAL so they are latched.
        self.ref_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # Storage for latest messages
        self.bad_readings_count = 0
        self.latest = {
            "unlabeled_markers":  None,
            "joint_states": None,
            "end_effector_pose": None,
            "fixed_ee_pose": None,
            "digit_D21118_image": None,
            "digit_D21122_image": None,
            "digit_D21123_image": None,
            "digit_D21124_image": None,
        }

        # Which keys we require before publishing a "full_data" sample
        self.required_keys = [
            "unlabeled_markers",
            "joint_states",
            "end_effector_pose",
            "fixed_ee_pose",
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
        
        # end_efactor_pose from kinova_state_pub
        self.sub_ee_pose = self.create_subscription(
            PoseStamped,
            "/end_effector_pose",
            self.cb_ee_pose,
            self.pose_qos,
        )
        self.sub_fixed_ee_pose = self.create_subscription(
            PoseStamped,
            "/natnet/fixed_ee_pose",
            self.cb_fixed_ee_pose,
            self.pose_qos,
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
            self.image_pub_qos,
        )

        # /full_data/digit/.../image_raw
        self.pub_digit_D21118_img = self.create_publisher(
            Image,
            "/full_data/digit/D21118/image_raw",
            self.image_pub_qos,
        )
        self.pub_digit_D21122_img = self.create_publisher(
            Image,
            "/full_data/digit/D21122/image_raw",
            self.image_pub_qos,
        )
        self.pub_digit_D21123_img = self.create_publisher(
            Image,
            "/full_data/digit/D21123/image_raw",
            self.image_pub_qos,
        )
        self.pub_digit_D21124_img = self.create_publisher(
            Image,
            "/full_data/digit/D21124/image_raw",
            self.image_pub_qos,
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
        if self.latest['unlabeled_markers'] is not None:
            last_time = self.latest['unlabeled_markers'].header.stamp
        else:
            last_time = None
        points = msg.points
        count = len(points)

        if count != self.num_markers:
            self.get_logger().warn(
                f"Expected {self.num_markers} markers, but got {count}."
            )
            self.bad_readings_count += 1
            if self.bad_readings_count >= 10:
                self.get_logger().error(
                    f"Received {self.bad_readings_count} consecutive bad readings on unlabeled_markers. Resetting stored data."
                )
                self.latest["unlabeled_markers"] = None
        else:
            self.latest["unlabeled_markers"] = msg
            self.bad_readings_count = 0
        
        # # log the time 
        # cur_time = msg.header.stamp
        # # the diff in seconds from cur to last
        # if self.latest['unlabeled_markers'] is not None and last_time is not None:
        #     delta_sec = (
        #         (cur_time.sec - last_time.sec) + 
        #         (cur_time.nanosec - last_time.nanosec) / 1e9
        #     )
        #     self.get_logger().info(
        #         f"Unlabeled markers received. Time since last: {delta_sec:.4f} s"
        #     )

    def cb_joint_states(self, msg: JointState):
        self.latest["joint_states"] = msg



    def cb_fixed_ee_pose(self, msg: PoseStamped):
        self.latest["fixed_ee_pose"] = msg
    
    def cb_ee_pose(self, msg: PoseStamped):
        self.latest["end_effector_pose"] = msg
    
    def cb_digit_D21118_image(self, msg: Image):
        self.latest["digit_D21118_image"] = msg

    def cb_digit_D21122_image(self, msg: Image):
        self.latest["digit_D21122_image"] = msg

    def cb_digit_D21123_image(self, msg: Image):
        
        if msg is None:
            return
        if self.latest["digit_D21123_image"] is not None:
            last_time = self.latest["digit_D21123_image"].header.stamp
        else:
            last_time = None
        
        
        self.latest["digit_D21123_image"] = msg

        # # log the time 
        # cur_time = msg.header.stamp
        # # the diff in seconds from cur to last
        # if self.latest['digit_D21123_image'] is not None and last_time is not None:
        #     delta_sec = (
        #         (cur_time.sec - last_time.sec) + 
        #         (cur_time.nanosec - last_time.nanosec) / 1e9
        #     )
        #     self.get_logger().info(
        #         f"Digit D21123 image received. Time since last: {delta_sec:.4f} s"
        #     )
    def cb_digit_D21124_image(self, msg: Image):
        if msg is None:
            return
        if self.latest["digit_D21124_image"] is not None:
            last_time = self.latest["digit_D21124_image"].header.stamp
        else:
            last_time = None
        
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
                self.get_logger().debug(f"Missing key: {key}")
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
        s_time = self.get_clock().now()
        # 1. Acquire Master Data
        # master_msg = self.latest["digit_D21118_image"] # The Master
        master_msg = self.latest['unlabeled_markers']
        
        if master_msg is None:
            self.get_logger().info("Master unlabeled_markers is None.")
            return 

        # 2. Extract Master Time
        # This effectively becomes the "capture time" for the whole bundle
        master_stamp = Time.from_msg(master_msg.header.stamp)

        # 3. Check Synchrony
        # Threshold: e.g., 0.1s. If sensors drift more than 10ms apart, discard.
        if not self.is_bundle_synchronized(master_stamp, threshold_sec=0.04):
            return
        e_time = self.get_clock().now()
        # self.get_logger().info(f"Synchronized bundle acquired in {(e_time - s_time).nanoseconds / 1e9:.6f} s")
        # 4. Publish Bundle using Master Time
        s_time = self.get_clock().now()
        self.publish_full_data(master_stamp)
        e_time = self.get_clock().now()
        # self.get_logger().info(f"Published full_data bundle in {(e_time - s_time).nanoseconds / 1e9:.6f} s")
        
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
                
                if delta_sec < 0.06: # 60ms tolerance
                    return latest_tf
                else:
                    self.get_logger().warn(f"Fallback TF too old: {delta_sec:.4f}s diff")
                    return None
                    
            except Exception:
                return None


    def _clone_image_with_new_stamp(self, img: Image, stamp_msg):
        """Shallow-clone Image while reusing img.data to avoid huge copies."""
        out = Image()
        # --- header ---
        out.header.stamp = stamp_msg
        out.header.frame_id = img.header.frame_id  # keep original frame_id
        # --- metadata ---
        out.height = img.height
        out.width = img.width
        out.encoding = img.encoding
        out.is_bigendian = img.is_bigendian
        out.step = img.step
        # --- payload (reuse buffer, do NOT deepcopy) ---
        out.data = img.data
        return out
    def publish_full_data(self, stamp: Time):
        """
        Publish data. 
        CRITICAL: We look up TF at `stamp` (master time), not `Time()` (latest).
        """
        stamp_msg = stamp.to_msg()
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

        # poses 
        fixed_ee_pose_msg = copy.deepcopy(self.latest["fixed_ee_pose"])
        fixed_ee_pose_msg.header.stamp = stamp_msg
        self.pub_natnet_fixed_pose.publish(fixed_ee_pose_msg)
        end_effector_pose_msg = copy.deepcopy(self.latest["end_effector_pose"])
        end_effector_pose_msg.header.stamp = stamp_msg
        self.pub_natnet_robot_ee_pose.publish(end_effector_pose_msg)
        # 3. Images
        # We publish the images with the MASTER stamp. 
        for _, key, pub in [
            ("D21118", "digit_D21118_image", self.pub_digit_D21118_img),
            ("D21122", "digit_D21122_image", self.pub_digit_D21122_img),
            ("D21123", "digit_D21123_image", self.pub_digit_D21123_img),
            ("D21124", "digit_D21124_image", self.pub_digit_D21124_img),
        ]:
            img = self.latest.get(key)
            if img is None:
                return
            pub.publish(self._clone_image_with_new_stamp(img, stamp_msg))
            
            

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
