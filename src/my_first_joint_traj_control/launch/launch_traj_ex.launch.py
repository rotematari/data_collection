from launch_ros.actions import Node
from launch import LaunchDescription

controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_trajectory_controller"],
),


def generate_launch_description():
    return LaunchDescription([
        controller_spawner
    ])