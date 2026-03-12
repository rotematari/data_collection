#!/usr/bin/env python3
"""Launch NJC nodes (dlo_spline_node + njc_controller_node).

Assumes data_collection_bringup is already running (natnet_pub, kinova_state_pub, etc.).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
    njc_share = get_package_share_directory('njc_ros')
    params_file = join(njc_share, 'config', 'njc_params.yaml')

    model_dir_arg = DeclareLaunchArgument(
        'model_dir',
        description='Path to the NJC model directory (must contain config.yaml + checkpoint)',
    )

    dlo_spline_node = Node(
        package='njc_ros',
        executable='dlo_spline_node',
        name='dlo_spline_node',
        output='screen',
        parameters=[params_file],
    )

    njc_controller_node = Node(
        package='njc_ros',
        executable='njc_controller_node',
        name='njc_controller_node',
        output='screen',
        parameters=[
            params_file,
            {'model_dir': LaunchConfiguration('model_dir')},
        ],
    )

    return LaunchDescription([
        model_dir_arg,
        dlo_spline_node,
        njc_controller_node,
    ])
