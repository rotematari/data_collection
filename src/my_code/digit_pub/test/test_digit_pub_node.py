# import os
# import sys
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
# import time

from digit_pub.digit_pub_node import DigitPublisherNode  # Adjust if path/module is different


class ImageTestSubscriber(Node):
    def __init__(self):
        super().__init__('test_image_sub')
        self.image_msg = None
        self.subscription = self.create_subscription(
            Image, 'digit/image_raw', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.image_msg = msg

@pytest.mark.rostest
def test_digit_publisher_node_publishes():
    rclpy.init()
    pub_node = DigitPublisherNode()
    sub_node = ImageTestSubscriber()

    try:
        # Give time for publisher to start and send a message
        for i in range(30):  # Wait up to 3 seconds
            rclpy.spin_once(sub_node, timeout_sec=0.1)
            if sub_node.image_msg is not None:
                break
        assert sub_node.image_msg is not None, "No image message received!"
        assert isinstance(sub_node.image_msg, Image)
    finally:
        pub_node.destroy_node()
        sub_node.destroy_node()
        rclpy.shutdown()
