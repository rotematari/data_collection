#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robotiq_msgs.msg import RobotiqGripperCommand, RobotiqGripperStatus
from std_msgs.msg import String
import sys
import os

# Add the pyRobotiqGripper to Python path
sys.path.append(os.path.expanduser('~/pyrobotiqgripper'))
try:
    from pyrobotiqgripper import RobotiqGripper
except ImportError:
    print("Could not import pyrobotiqgripper. Please install it first.")
    sys.exit(1)

class RobotiqGripperNode(Node):
    def __init__(self):
        super().__init__('robotiq_gripper_node')
        
        # Parameters
        self.declare_parameter('device', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        # self.declare_parameter('status_rate', 10.0)
        
        device = self.get_parameter('device').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        # status_rate = self.get_parameter('status_rate').get_parameter_value().double_value
        
        # Initialize gripper
        try:
            self.gripper = RobotiqGripper()
            self.get_logger().info(f"Connected to Robotiq gripper on {device}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to gripper: {e}")
            return
        
        # Publishers
        # self.status_pub = self.create_publisher(
        #     RobotiqGripperStatus, 
        #     'robotiq_gripper/status', 
        #     10
        # )
        
        # Subscribers
        self.command_sub = self.create_subscription(
            RobotiqGripperCommand,
            'robotiq_gripper/command',
            self.command_callback,
            10
        )
        
        # # Status timer
        # self.status_timer = self.create_timer(
        #     1.0 / status_rate, 
        #     self.publish_status
        # )
        
        self.get_logger().info("Robotiq gripper node started")
    
    def command_callback(self, msg: RobotiqGripperCommand):
        """Handle gripper commands"""
        try:
            if msg.command == "activate":
                self.gripper.activate()
                self.get_logger().info("Activating gripper")
                
            elif msg.command == "reset":
                self.gripper.reset()
                self.get_logger().info("Resetting gripper")
                
            elif msg.command == "open":
                self.gripper.open()
                self.get_logger().info("Opening gripper")
                
            elif msg.command == "close":
                self.gripper.close()
                self.get_logger().info("Closing gripper")
                
            elif msg.command == "goto":
                self.gripper.goTo(msg.position, msg.speed, msg.force)
                self.get_logger().info(f"Moving to position {msg.position}")
                
            else:
                self.get_logger().warn(f"Unknown command: {msg.command}")
                
        except Exception as e:
            self.get_logger().error(f"Command failed: {e}")
    
    # def publish_status(self):
    #     """Publish gripper status"""
    #     try:
    #         status = RobotiqGripperStatus()
            
    #         # Get status from gripper
    #         gripper_status = self.gripper.get_status()
            
    #         status.is_active = gripper_status.get('gACT', 0) == 1
    #         status.is_moving = gripper_status.get('gGTO', 0) == 1
    #         status.object_detected = gripper_status.get('gOBJ', 0) > 0
    #         status.position_request = gripper_status.get('gPR', 0)
    #         status.position_actual = gripper_status.get('gPO', 0)
    #         status.current = gripper_status.get('gCU', 0)
            
    #         self.status_pub.publish(status)
            
    #     except Exception as e:
    #         self.get_logger().warn(f"Failed to get status: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotiqGripperNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()