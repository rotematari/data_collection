from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg._string import String
from digit_pub.digit_array import DigitArray
from cv_bridge import CvBridge


class DigitPublisherNode(Node):
    def __init__(self):
        super().__init__("digit_pub_node")
        self.declare_parameter("rate", 30.0)
        self.declare_parameter("diff_with_ref", False)
        self.declare_parameter("resolution", "QVGA")
        self.declare_parameter("fps", 30)
        self.declare_parameter("intensity", 15)

        self.bridge = CvBridge()
        self.digit_array = DigitArray(
            show_log=True,
            ros_logger=self.get_logger(),
            resolution=self.get_parameter("resolution").get_parameter_value().string_value,
            fps=self.get_parameter("fps").get_parameter_value().integer_value,
            intensity=self.get_parameter("intensity").get_parameter_value().integer_value
        )
        self.digit_publishers: List = []
        self._set_up_publishers()
        self.rate = self.get_parameter("rate").get_parameter_value().double_value
        self.diff_with_ref = self.get_parameter("diff_with_ref").get_parameter_value().bool_value

        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
        self.get_logger().info("DigitPublisherNode started.with rate: {}".format(self.rate))
        
        self.get_logger().info(f"diff_with_ref is set to: {self.diff_with_ref}")

        # # setup a sub to save the reference frame
        # self.create_subscription(String, "digit_cmd", self.cmd_callback, 10)

    def timer_callback(self):

        if len(self.digit_array.digits) == 0:
            # self.get_logger().warn("No DIGIT devices connected.")
            return
        for i, digit in enumerate(self.digit_array.digits):
            try:
                if self.diff_with_ref:
                    ref_frame = self.digit_array.get_reference_frame(digit.serial)
                    if ref_frame is not None:
                        frame = digit.get_diff(ref_frame)
                    else:
                        frame = digit.get_frame()
                else:
                    frame = digit.get_frame()
                
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.digit_publishers[i].publish(msg)
            except Exception as e:
                self.get_logger().error(f"Failed to get or publish frame: {e}")
                digit.disconnect()
                self.get_logger().info(f"Disconnected from {digit.serial}")
                print("Attempting to reconnect...")
                try:
                    digit.connect()
                    self.get_logger().info(f"Reconnected to {digit.serial}")
                except Exception as e:
                    self.get_logger().error(f"Failed to reconnect to {digit.serial}: {e}")

    
    # def cmd_callback(self, msg: String):
    #     if msg.data == "save_ref":
    #         self.get_logger().info("Saving reference frames for all connected DIGIT devices.")
    #         self.digit_array.capture_all_reference_frames()
    #         self.get_logger().info("Reference frames saved.")
    def _set_up_publishers(self):
        for i in range(len(self.digit_array.digits)):
            topic_name = f"digit_{i}/image_raw"
            publisher = self.create_publisher(Image, topic_name, 10)
            self.digit_publishers.append(publisher)
            self.get_logger().info(f"Publisher created for {topic_name} with DIGIT devices: {self.digit_array.digits}")

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
