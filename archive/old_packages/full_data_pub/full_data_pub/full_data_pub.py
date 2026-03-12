#!/usr/bin/env python3
"""
FullDataPub with separate callback groups + MultiThreadedExecutor.

Goal:
- Subscriber callbacks keep running even if timer_callback() is heavy.
- Timer callback runs in its own callback group.
"""

import copy
import numpy as np

import rclpy
from rclpy.node import Node

# --- ADDED: callback groups + executor ---
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState, Image
from visualization_msgs.msg import Marker

from tf2_ros import Buffer, TransformListener
import tf_transformations


class FullDataPub(Node):
    def __init__(self):
        super().__init__("full_data_pub")

        # ---------------- Parameters ----------------
        self.declare_parameter("rate", 10)
        self.declare_parameter("num_markers", 12)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("t_calib", [0.0, 0.0, 0.0])
        self.declare_parameter("q_calib", [0.0, 0.0, 0.0, 1.0])  # x,y,z,w

        self.rate = self.get_parameter("rate").get_parameter_value().integer_value
        self.num_markers = self.get_parameter("num_markers").get_parameter_value().integer_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.t_calib = self.get_parameter("t_calib").get_parameter_value().double_array_value
        self.q_calib = self.get_parameter("q_calib").get_parameter_value().double_array_value

        self.get_logger().info(f"FullDataPub node started with rate: {self.rate} Hz")

        # ---------------- QoS ----------------
        self.image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pose_qos_pub = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.ref_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # ---------------- Storage ----------------
        self.bad_readings_count = 0
        self.latest = {
            "unlabeled_markers": None,
            "joint_states": None,
            "end_effector_pose": None,
            "fixed_ee_pose": None,
            "digit_D21118_image": None,
            "digit_D21122_image": None,
            "digit_D21123_image": None,
            "digit_D21124_image": None,
        }
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

        self.ref_published = {"D21118": False, "D21122": False, "D21123": False, "D21124": False}

        # ---------------- TF ----------------
        self.buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.listener = TransformListener(self.buffer, self)

        # ============================================================
        # --- ADDED: Callback groups to prevent timer blocking subs ---
        # ============================================================
        # Use Reentrant for subscriptions so multiple subs can run concurrently.
        self.sub_cb_group = ReentrantCallbackGroup()
        # Use MutuallyExclusive for timer to avoid overlapping timer executions.
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()

        # ---------------- Subscriptions / Publishers ----------------
        self.set_up_subscriptions()
        self.set_up_publishers()

        # ============================================================
        # --- CHANGED: Timer created in its own callback group ---
        # ============================================================
        self.timer = self.create_timer(
            1.0 / self.rate,
            self.timer_callback,
            callback_group=self.timer_cb_group,
        )

    def set_up_subscriptions(self):
        # IMPORTANT: all subscriptions in sub_cb_group so they can keep flowing
        self.sub_unlabeled_markers = self.create_subscription(
            Marker,
            "/natnet/unlabeled_marker_data",
            self.cb_unlabeled_markers,
            self.image_qos,
            callback_group=self.sub_cb_group,  # ADDED
        )
        self.sub_joint_state = self.create_subscription(
            JointState,
            "/joint_states",
            self.cb_joint_states,
            self.image_qos,
            callback_group=self.sub_cb_group,  # ADDED
        )

        self.sub_digit_D21118_img = self.create_subscription(
            Image,
            "/digit/D21118/image_raw",
            self.cb_digit_D21118_image,
            self.image_qos,
            callback_group=self.sub_cb_group,  # ADDED
        )
        self.sub_digit_D21122_img = self.create_subscription(
            Image,
            "/digit/D21122/image_raw",
            self.cb_digit_D21122_image,
            self.image_qos,
            callback_group=self.sub_cb_group,  # ADDED
        )
        self.sub_digit_D21123_img = self.create_subscription(
            Image,
            "/digit/D21123/image_raw",
            self.cb_digit_D21123_image,
            self.image_qos,
            callback_group=self.sub_cb_group,  # ADDED
        )
        self.sub_digit_D21124_img = self.create_subscription(
            Image,
            "/digit/D21124/image_raw",
            self.cb_digit_D21124_image,
            self.image_qos,
            callback_group=self.sub_cb_group,  # ADDED
        )

        # Ref images (latched-like) - also in sub_cb_group
        self.sub_digit_D21118_ref = self.create_subscription(
            Image,
            "/digit/D21118/ref_image",
            self.cb_digit_ref_factory("D21118"),
            self.ref_qos,
            callback_group=self.sub_cb_group,  # ADDED
        )
        self.sub_digit_D21122_ref = self.create_subscription(
            Image,
            "/digit/D21122/ref_image",
            self.cb_digit_ref_factory("D21122"),
            self.ref_qos,
            callback_group=self.sub_cb_group,  # ADDED
        )
        self.sub_digit_D21123_ref = self.create_subscription(
            Image,
            "/digit/D21123/ref_image",
            self.cb_digit_ref_factory("D21123"),
            self.ref_qos,
            callback_group=self.sub_cb_group,  # ADDED
        )
        self.sub_digit_D21124_ref = self.create_subscription(
            Image,
            "/digit/D21124/ref_image",
            self.cb_digit_ref_factory("D21124"),
            self.ref_qos,
            callback_group=self.sub_cb_group,  # ADDED
        )

        self.sub_ee_pose = self.create_subscription(
            PoseStamped,
            "/end_effector_pose",
            self.cb_ee_pose,
            self.pose_qos,
            callback_group=self.sub_cb_group,  # ADDED
        )
        self.sub_fixed_ee_pose = self.create_subscription(
            PoseStamped,
            "/natnet/fixed_ee_pose",
            self.cb_fixed_ee_pose,
            self.pose_qos,
            callback_group=self.sub_cb_group,  # ADDED
        )

    def set_up_publishers(self):
        self.pub_natnet_fixed_pose = self.create_publisher(
            PoseStamped, "/full_data/natnet_fixed_pose", self.pose_qos_pub
        )
        self.pub_natnet_robot_ee_pose = self.create_publisher(
            PoseStamped, "/full_data/natnet_robot_ee_pose", self.pose_qos_pub
        )
        self.pub_unlabeled_markers = self.create_publisher(
            Marker, "/full_data/natnet/unlabeled_marker_data", self.pose_qos_pub
        )

        self.pub_joint_states = self.create_publisher(
            JointState, "/full_data/joint_states", self.image_pub_qos
        )

        self.pub_digit_D21118_img = self.create_publisher(
            Image, "/full_data/digit/D21118/image_raw", self.image_pub_qos
        )
        self.pub_digit_D21122_img = self.create_publisher(
            Image, "/full_data/digit/D21122/image_raw", self.image_pub_qos
        )
        self.pub_digit_D21123_img = self.create_publisher(
            Image, "/full_data/digit/D21123/image_raw", self.image_pub_qos
        )
        self.pub_digit_D21124_img = self.create_publisher(
            Image, "/full_data/digit/D21124/image_raw", self.image_pub_qos
        )

        self.pub_digit_D21118_ref = self.create_publisher(
            Image, "/full_data/digit/D21118/ref_image", self.ref_qos
        )
        self.pub_digit_D21122_ref = self.create_publisher(
            Image, "/full_data/digit/D21122/ref_image", self.ref_qos
        )
        self.pub_digit_D21123_ref = self.create_publisher(
            Image, "/full_data/digit/D21123/ref_image", self.ref_qos
        )
        self.pub_digit_D21124_ref = self.create_publisher(
            Image, "/full_data/digit/D21124/ref_image", self.ref_qos
        )

    # ---------------- Callbacks ----------------
    def cb_unlabeled_markers(self, msg: Marker):
        if msg is None:
            return
        points = msg.points
        if len(points) != self.num_markers:
            self.bad_readings_count += 1
            if self.bad_readings_count >= 10:
                self.latest["unlabeled_markers"] = None
        else:
            self.latest["unlabeled_markers"] = msg
            self.bad_readings_count = 0

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
        self.latest["digit_D21123_image"] = msg

    def cb_digit_D21124_image(self, msg: Image):
        self.latest["digit_D21124_image"] = msg

    def cb_digit_ref_factory(self, serial: str):
        def cb(msg: Image):
            if self.ref_published[serial]:
                return

            pub = {
                "D21118": self.pub_digit_D21118_ref,
                "D21122": self.pub_digit_D21122_ref,
                "D21123": self.pub_digit_D21123_ref,
                "D21124": self.pub_digit_D21124_ref,
            }.get(serial, None)
            if pub is None:
                return

            pub.publish(msg)
            self.ref_published[serial] = True

        return cb

    # ---------------- Synchronization helpers ----------------
    def is_bundle_synchronized(self, master_stamp: Time, threshold_sec: float) -> bool:
        master_ns = master_stamp.nanoseconds
        for key in self.required_keys:
            msg = self.latest.get(key)
            if msg is None:
                # CHANGED: avoid INFO spam
                self.get_logger().debug(f"Missing key: {key}")
                return False

            msg_time_ns = Time.from_msg(msg.header.stamp).nanoseconds
            delta_sec = abs(master_ns - msg_time_ns) / 1e9
            if delta_sec > threshold_sec:
                return False
        return True

    def _clone_image_with_new_stamp(self, img: Image, stamp_msg):
        out = Image()
        out.header.stamp = stamp_msg
        out.header.frame_id = img.header.frame_id
        out.height = img.height
        out.width = img.width
        out.encoding = img.encoding
        out.is_bigendian = img.is_bigendian
        out.step = img.step
        out.data = img.data
        return out

    def timer_callback(self):
        master_msg = self.latest.get("unlabeled_markers")
        if master_msg is None:
            return

        master_stamp = Time.from_msg(master_msg.header.stamp)

        if not self.is_bundle_synchronized(master_stamp, threshold_sec=0.04):
            return

        self.publish_full_data(master_stamp)

    def publish_full_data(self, stamp: Time):
        stamp_msg = stamp.to_msg()

        # Markers: transform (kept as in your code)
        msg = copy.deepcopy(self.latest["unlabeled_markers"])

        R_natnet_world = tf_transformations.quaternion_matrix(
            [self.q_calib[0], self.q_calib[1], self.q_calib[2], self.q_calib[3]]
        )
        T_natnet_world = tf_transformations.translation_matrix(
            [self.t_calib[0], self.t_calib[1], self.t_calib[2]]
        )
        M_transform = tf_transformations.concatenate_matrices(T_natnet_world, R_natnet_world)

        msg.header.frame_id = self.base_frame
        for point in msg.points:
            p = np.array([point.x, point.y, point.z, 1.0])
            p2 = M_transform.dot(p)
            point.x = float(p2[0])
            point.y = float(p2[1])
            point.z = float(p2[2])

        msg.header.stamp = stamp_msg
        self.pub_unlabeled_markers.publish(msg)

        # Joint states
        js = copy.deepcopy(self.latest["joint_states"])
        js.header.stamp = stamp_msg
        self.pub_joint_states.publish(js)

        # Poses
        fixed_pose = copy.deepcopy(self.latest["fixed_ee_pose"])
        fixed_pose.header.stamp = stamp_msg
        self.pub_natnet_fixed_pose.publish(fixed_pose)

        ee_pose = copy.deepcopy(self.latest["end_effector_pose"])
        ee_pose.header.stamp = stamp_msg
        self.pub_natnet_robot_ee_pose.publish(ee_pose)

        # Images: publish with master stamp (NO SLEEP)
        for key, pub in [
            ("digit_D21118_image", self.pub_digit_D21118_img),
            ("digit_D21122_image", self.pub_digit_D21122_img),
            ("digit_D21123_image", self.pub_digit_D21123_img),
            ("digit_D21124_image", self.pub_digit_D21124_img),
        ]:
            img = self.latest.get(key)
            if img is None:
                return
            pub.publish(self._clone_image_with_new_stamp(img, stamp_msg))


def main(args=None):
    rclpy.init(args=args)

    node = FullDataPub()

    # ============================================================
    # --- ADDED: MultiThreadedExecutor so subs and timer run concurrently ---
    # ============================================================
    executor = MultiThreadedExecutor(num_threads=10)  # CHANGED: tune threads as needed
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
