#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand as GripperAction
# from my_msgs.msg import 

class GripperCmdBridge(Node):
    def __init__(self):
        super().__init__('gripper_cmd_bridge')

        # Parameters (tune in YAML/CLI)
        self.declare_parameter('action_name', '/gripper_controller/gripper_cmd')
        self.declare_parameter('open_position', 0.08)  # meters (example)
        self.declare_parameter('close_position', 0.00)
        self.declare_parameter('default_max_effort', 40.0)

        action_name = self.get_parameter('action_name').get_parameter_value().string_value
        self._client = ActionClient(self, GripperAction, action_name)

        self._sub = self.create_subscription(
            GripperCommand, '/gripper/cmd', self._on_cmd, 10
        )

        self.get_logger().info(f'GripperCmdBridge up. Subscribing /gripper/cmd → action "{action_name}"')

    def _on_cmd(self, msg: GripperCommand):
        if msg.mode == GripperCommand.STOP:
            self.get_logger().info('STOP received → canceling all goals')
            self._client.cancel_all_goals()
            return

        if not self._client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('Gripper action server not available')
            return

        # Decide position/effort
        if msg.mode == GripperCommand.OPEN:
            pos = msg.position if msg.position != 0.0 else self.get_parameter('open_position').value
            label = 'OPEN'
        elif msg.mode == GripperCommand.CLOSE:
            pos = msg.position if msg.position != 0.0 else self.get_parameter('close_position').value
            label = 'CLOSE'
        else:
            self.get_logger().warn(f'Unknown mode: {msg.mode}')
            return

        max_effort = msg.max_effort if msg.max_effort != 0.0 else self.get_parameter('default_max_effort').value

        goal = GripperAction.Goal()
        goal.command.position = float(pos)
        goal.command.max_effort = float(max_effort)

        self.get_logger().info(f'Sending {label}: pos={pos:.3f}, max_effort={max_effort:.1f}')
        # Fire-and-forget: don't await result
        self._client.send_goal_async(goal)

def main():
    rclpy.init()
    node = GripperCmdBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
