#!/usr/bin/env python3
"""Launch kinova end effector pose publisher and NatNet publisher."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import (
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from os.path import join


def safe_load_yaml(package_name, file_name):
    file_path = join(get_package_share_directory(package_name), 'config', file_name)
    return file_path

    
def generate_launch_description():
    natnet_cfg = safe_load_yaml('natnet_pub', 'data_collection_configs.yaml')
    digit_cfg = safe_load_yaml('data_collection_bringup', 'digit_config.yaml')
    print(f"Using NatNet config: {natnet_cfg}")
    print(f"Using DIGIT config: {digit_cfg}")
    kinova_node = Node(
        package='kinova_state_pub',
        executable='end_effector_pose_pub_node',
        name='end_effector_pose_pub_node',
        output='screen',
        parameters=[{
            'base_frame': 'base_link',
            'end_effector_frame': 'end_effector_link',
            'publish_topic': 'end_effector_pose',
        }]
    )

    # natnet_node = Node(
    #     package='natnet_pub',
    #     executable='natnet_client_pub_node',
    #     name='natnet_client_pub_node',
    #     output='screen',
    #     parameters= [natnet_cfg],
    # )
    digit_node = Node(
        package='digit_pub',
        executable='digit_pub_node',
        name='digit_pub_node',
        output='screen',
        parameters=[digit_cfg],
    )
    
    robotiq_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        join(
            get_package_share_directory('robotiq_rviz_plugin'),
            'launch',
            'robotiq_rviz.launch.py',
        )
    ),
    # launch_arguments={'device': '/dev/ttyUSB0', 'rviz': 'true'}.items(),  # optional overrides
)
    
    
    # description_package = "data_collection_bringup"
    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare(description_package), "rviz", "data_collection_config.rviz"]
    # )
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    # )
    # delay_rviz_node = TimerAction(period=2.0, actions=[rviz_node])
    return LaunchDescription([
        kinova_node,
        # natnet_node,
        digit_node,
        robotiq_launch,
    ])
