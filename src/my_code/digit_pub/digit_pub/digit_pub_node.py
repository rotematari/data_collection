from typing import List, Dict, Any

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from digit_pub.digit_array import DigitArray
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import numpy as np


class DigitPublisherNode(Node):
    def __init__(self):
        super().__init__("digit_pub_node")
        self.declare_parameter("rate", 30.0)
        self.declare_parameter("diff_with_ref", False)
        self.declare_parameter("resolution", "VGA")
        self.declare_parameter("fps", "high")  # "high" or "low"
        self.declare_parameter("intensity", 15)
        
        self.fps = self.get_parameter("fps").get_parameter_value().string_value
        self.resolution = self.get_parameter("resolution").get_parameter_value().string_value
        if self.resolution == "VGA":
            if self.fps == "high":
                self.fps = 30
            else:
                self.fps = 15
        else:  # QVGA
            if self.fps == "high":
                self.fps = 60
            else:
                self.fps = 30
        
        # self.rate = int(self.fps*2)  # publish at double the fps
        # self.rate = self.get_parameter("rate").get_parameter_value().double_value
        self.rate = min(
                        self.get_parameter("rate").get_parameter_value().double_value,
                        float(self.fps),
                    )
        self.diff_with_ref = self.get_parameter("diff_with_ref").get_parameter_value().bool_value
        
        self.intesity = self.get_parameter("intensity").get_parameter_value().integer_value
        
        self.bridge = CvBridge()
        self.digit_array = DigitArray(
            show_log=True,
            ros_logger=self.get_logger(),
            resolution=self.resolution,
            fps=self.fps,
            intensity=self.intesity
        )
        self.digit_publishers: List = []

        # Latched-like QoS for late joiners (rosbag2, rviz, etc.)
        # -------- QoS --------

        # "Latched-like" QoS for reference frames so late subscribers & rosbag2 get the last sample
        self.ref_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.live_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2
        )
        # -------- Publishers --------
        self.live_publishers: Dict[str, Any] = {}  # serial -> pub
        self.ref_publishers: Dict[str, Any ] = {}   # serial -> pub
        self._set_up_publishers()

        # -------- Timer for live frames --------
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
        # self.get_logger().info(f"DigitPublisherNode started. rate={self.rate}, diff_with_ref={self.diff_with_ref}")
        
        # log the parameters
        self.get_logger().info(f"DIGIT resolution: {self.resolution},\n fps: {self.fps},\n intensity: {self.intesity}")
        # -------- Command sub --------
        self.create_subscription(String, "digit_cmd", self.cmd_callback, 10)
        self.pub_ref_once()  # publish ref frames once at startup
        

    def timer_callback(self):
        
        
        if len(self.digit_array.digits) == 0:
            # self.get_logger().warn("No DIGIT devices connected.")
            return
        
        for digit in self.digit_array.digits:
            serial = digit.serial
            if serial not in self.live_publishers:
                self.get_logger().warn(f"No publisher for {serial}")
                continue
            try:
                if self.diff_with_ref:
                    ref_frame = self.digit_array.get_reference_frame(digit.serial)
                    if ref_frame is not None:
                        frame = digit.get_diff(ref_frame)
                    else:
                        frame = digit.get_frame()
                else:

                    frame = digit.get_frame()
                
                frame = np.ascontiguousarray(frame)
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = serial
                self.live_publishers[serial].publish(msg)

            except Exception as e:
                self.get_logger().error(f"Failed to get or publish frame: {e}")
                digit.disconnect()
                self.get_logger().info(f"Disconnected from {digit.serial}")
                print("Attempting to reconnect...")
                try:
                    digit.connect()
                    _ = digit.get_frame()  # verify connection
                    self.get_logger().info(f"Reconnected to {digit.serial}")
                except Exception as e:
                    self.get_logger().error(f"Failed to reconnect to {digit.serial}: {e}")

    def _publish_ref_for_serial(self, serial: str) -> bool:
        """Publish reference frame for a single DIGIT sensor.
        
        Returns:
            True if published successfully, False otherwise.
        """
        ref = self.digit_array.get_reference_frame(serial)
        if ref is None:
            self.get_logger().warn(f"No reference frame available for {serial}")
            return False

        imsg = self.bridge.cv2_to_imgmsg(ref, encoding="bgr8")
        imsg.header.stamp = self.get_clock().now().to_msg()
        imsg.header.frame_id = serial
        self.ref_publishers[serial].publish(imsg)
        self.get_logger().info(f"Published ref (raw) for {serial}")
        return True

    def pub_ref_once(self):
        """Publish reference frames exactly once per connected DIGIT."""
        for digit in self.digit_array.digits:
            self._publish_ref_for_serial(digit.serial)

    # ---------------- Command handling ----------------
    def cmd_callback(self, msg: String):
        if msg.data == "save_ref":
            self.get_logger().info("Capturing and publishing reference frames for all DIGIT devices...")
            try:
                self.digit_array.capture_all_reference_frames()
            except Exception as e:
                self.get_logger().error(f"capture_all_reference_frames() failed: {e}")

            # Publish exactly once per sensor (TRANSIENT_LOCAL keeps it available)
            for digit in self.digit_array.digits:
                self._publish_ref_for_serial(digit.serial)
                
    def _set_up_publishers(self):
        # Create live and ref publishers per connected DIGIT
        for digit in self.digit_array.digits:
            serial = digit.serial
            # Live topic per serial
            live_topic = f"/digit/{serial}/image_raw"
            self.live_publishers[serial] = self.create_publisher(Image, live_topic, self.live_qos)
            self.get_logger().info(f"Live publisher: {live_topic}")

            ref_topic = f"/digit/{serial}/ref_image"
            self.ref_publishers[serial] = self.create_publisher(Image, ref_topic, self.ref_qos)
            self.get_logger().info(f"Ref publisher (latched): {ref_topic}")

    def destroy_node(self):
        self.digit_array.disconnect_all()
        self.get_logger().info("Disconnected from DIGIT.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DigitPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
