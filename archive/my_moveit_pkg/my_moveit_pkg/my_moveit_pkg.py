#!/usr/bin/env python3
"""
A minimal-yet-robust MoveIt 2 node that moves a 7‑DoF arm to either
- a named joint target (from SRDF), or
- a Cartesian pose target (geometry_msgs/PoseStamped in base frame).

Tested with ROS 2 Jazzy + MoveIt 2 using `moveit_py`.

Usage examples:
  # Named target from SRDF (e.g., "home")
  ros2 run my_moveit_pkg moveit_pose_goal_node --ros-args -p target_type:="named" -p named_target:="home"

  # Pose target
  ros2 run my_moveit_pkg moveit_pose_goal_node --ros-args \
    -p target_type:="pose" \
    -p pose_xyz:='[0.4, -0.2, 0.25]' \
    -p pose_rpy_deg:='[180.0, 0.0, 90.0]' \
    -p base_frame:="base_link"

  # Joint target explicitly
  ros2 run my_moveit_pkg moveit_pose_goal_node --ros-args -p target_type:="joints" -p joint_values:='[0, -1.0, 0, -2.1, 0, 2.1, 0.7]'

Parameters:
  planning_group     (string, default: "manipulator")
  target_type        (string, one of: "named", "pose", "joints")
  named_target       (string, used when target_type=="named")
  pose_xyz           (double[3], meters, used when target_type=="pose")
  pose_rpy_deg       (double[3], degrees, used when target_type=="pose")
  base_frame         (string, default: "base_link")
  joint_values       (double[N], used when target_type=="joints")
  cartesian_link     (string, default: end effector link inferred from SRDF)
  vel_scaling        (double, 0..1, default: 0.2)
  acc_scaling        (double, 0..1, default: 0.2)
  plan_only          (bool, default: False)
  allow_replanning   (bool, default: True)
  planning_time      (double, seconds, default: 5.0)
  num_attempts       (int, default: 3)

Prereqs:
  - robot_description & SRDF are available (e.g., via your bringup)
  - /joint_states and TF tree are being published

"""

from typing import List
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time

# MoveItPy (MoveIt 2 Python API)

from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from moveit.core.robot_model import RobotModelConst


def rpy_deg_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float):
    """Convert RPY degrees to quaternion (x,y,z,w)."""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    # ZYX intrinsic
    cy = math.cos(y * 0.5)
    sy = math.sin(y * 0.5)
    cp = math.cos(p * 0.5)
    sp = math.sin(p * 0.5)
    cr = math.cos(r * 0.5)
    sr = math.sin(r * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


class MoveItPoseGoalNode(Node):
    def __init__(self):
        super().__init__("moveit_pose_goal_node")

        # Declare parameters with defaults
        self.declare_parameter("planning_group", "manipulator")
        self.declare_parameter("target_type", "named")
        self.declare_parameter("named_target", "home")
        self.declare_parameter("pose_xyz", [0.4, 0.0, 0.3])
        self.declare_parameter("pose_rpy_deg", [180.0, 0.0, 0.0])
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("joint_values", [])
        self.declare_parameter("cartesian_link", "")
        self.declare_parameter("vel_scaling", 0.2)
        self.declare_parameter("acc_scaling", 0.2)
        self.declare_parameter("plan_only", False)
        self.declare_parameter("allow_replanning", True)
        self.declare_parameter("planning_time", 5.0)
        self.declare_parameter("num_attempts", 3)

        # Read params
        self.group_name: str = self.get_parameter("planning_group").get_parameter_value().string_value
        self.target_type: str = self.get_parameter("target_type").get_parameter_value().string_value
        self.named_target: str = self.get_parameter("named_target").get_parameter_value().string_value
        self.pose_xyz: List[float] = self.get_parameter("pose_xyz").get_parameter_value().double_array_value
        self.pose_rpy_deg: List[float] = self.get_parameter("pose_rpy_deg").get_parameter_value().double_array_value
        self.base_frame: str = self.get_parameter("base_frame").get_parameter_value().string_value
        self.joint_values: List[float] = self.get_parameter("joint_values").get_parameter_value().double_array_value
        self.cartesian_link: str = self.get_parameter("cartesian_link").get_parameter_value().string_value
        self.vel_scaling: float = self.get_parameter("vel_scaling").get_parameter_value().double_value
        self.acc_scaling: float = self.get_parameter("acc_scaling").get_parameter_value().double_value
        self.plan_only: bool = self.get_parameter("plan_only").get_parameter_value().bool_value
        self.allow_replanning: bool = self.get_parameter("allow_replanning").get_parameter_value().bool_value
        self.planning_time: float = self.get_parameter("planning_time").get_parameter_value().double_value
        self.num_attempts: int = self.get_parameter("num_attempts").get_parameter_value().integer_value

        # Create MoveItPy context and planning component
        self.moveit = MoveItPy(node_name=self.get_name())
        self.pc = self.moveit.get_planning_component(self.group_name)

        # Configure planning settings
        self.pc.set_max_velocity_scaling_factor(self.vel_scaling)
        self.pc.set_max_acceleration_scaling_factor(self.acc_scaling)
        self.pc.set_planning_time(self.planning_time)
        self.pc.allow_replanning(self.allow_replanning)

        self.get_logger().info(f"Using planning group: {self.group_name}")
        self.get_logger().info(f"Target type: {self.target_type}")

        # Kick off once after a short timer so logging shows up
        self.create_timer(0.5, self._run_once)
        self._ran = False

    def _run_once(self):
        if self._ran:
            return
        self._ran = True
        try:
            if self.target_type == "named":
                self._go_to_named()
            elif self.target_type == "pose":
                self._go_to_pose()
            elif self.target_type == "joints":
                self._go_to_joints()
            else:
                raise ValueError(f"Unknown target_type: {self.target_type}")
        except Exception as e:
            self.get_logger().error(f"Planning/execution failed: {e}")
        finally:
            # Shutdown after one attempt to behave like a one-shot tool
            rclpy.shutdown()

    # ---- Target helpers --------------------------------------------------
    def _go_to_named(self):
        self.get_logger().info(f"Setting named target: {self.named_target}")
        self.pc.set_start_state_to_current_state()
        self.pc.set_goal_state(configuration_name=self.named_target)
        self._plan_and_execute()

    def _go_to_joints(self):
        if not self.joint_values:
            raise ValueError("joint_values is empty; provide N values matching the group joints.")
        self.get_logger().info(f"Setting joint target: {self.joint_values}")
        self.pc.set_start_state_to_current_state()
        self.pc.set_goal_state(joint_positions=self.joint_values)
        self._plan_and_execute()

    def _go_to_pose(self):
        if len(self.pose_xyz) != 3 or len(self.pose_rpy_deg) != 3:
            raise ValueError("pose_xyz and pose_rpy_deg must be length-3 lists.")

        qx, qy, qz, qw = rpy_deg_to_quat(*self.pose_rpy_deg)
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(self.pose_xyz[0])
        pose.pose.position.y = float(self.pose_xyz[1])
        pose.pose.position.z = float(self.pose_xyz[2])
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        # Pick link: user-provided or EE link from SRDF
        link = self.cartesian_link or self._infer_ee_link()
        self.get_logger().info(f"Setting pose target on link '{link}' in frame '{self.base_frame}'")

        self.pc.set_start_state_to_current_state()
        self.pc.set_goal_state(pose_stamped_msg=pose, pose_link=link)
        self._plan_and_execute()

    def _infer_ee_link(self) -> str:
        # Try SRDF end effector for this group; fallback to last link in chain
        model: RobotModelConst = self.moveit.get_robot_model()
        joint_model_group = model.get_joint_model_group(self.group_name)
        if joint_model_group is None:
            raise RuntimeError(f"No joint model group '{self.group_name}' in SRDF")
        tip_links = joint_model_group.get_link_model_names()
        if not tip_links:
            raise RuntimeError(f"Group '{self.group_name}' has no links")
        return tip_links[-1]

    def _plan_and_execute(self):
        success = False
        last_plan = None
        for i in range(max(1, self.num_attempts)):
            self.get_logger().info(f"Planning attempt {i+1}/{self.num_attempts}…")
            plan_res = self.pc.plan()
            if not plan_res or plan_res.trajectory is None or plan_res.error_code.val != 1:
                self.get_logger().warn("Planning failed; retrying…")
                continue
            last_plan = plan_res.trajectory
            success = True
            break

        if not success:
            raise RuntimeError("Planning failed after attempts")

        if self.plan_only:
            self.get_logger().info("Plan-only mode: NOT executing trajectory")
            return

        # Execute with the MoveItPy planning scene/trajectory execution manager
        exec_res = self.moveit.execute(self.group_name, last_plan, blocking=True)
        if not exec_res:
            raise RuntimeError("Execution failed")
        self.get_logger().info("Motion completed successfully ✅")


def main():
    rclpy.init()
    node = MoveItPoseGoalNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
