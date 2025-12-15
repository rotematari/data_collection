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
    bringup_cfg = safe_load_yaml('data_collection_bringup', 'data_collection_bringup.yaml')
    print(f"Using Bringup config: {bringup_cfg}")
    
    
    
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

    natnet_node = Node(
        package='natnet_pub',
        executable='natnet_client_pub_node',
        name='natnet_client_pub_node',
        output='screen',
        parameters= [bringup_cfg],
    )
    digit_node = Node(
        package='digit_pub',
        executable='digit_pub_node',
        name='digit_pub_node',
        output='screen',
        parameters=[bringup_cfg],
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

    # full data publisher with some delay to allow other nodes to start
    full_data_pub_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='full_data_pub',
                executable='full_data_pub',
                name='full_data_pub',
                output='screen',
                parameters=[bringup_cfg],
            )
        ]
    )
    
    
    return LaunchDescription([
        kinova_node,
        natnet_node,
        digit_node,
        robotiq_launch,
        # delay_rviz_node,
        full_data_pub_node,
        
    ])
