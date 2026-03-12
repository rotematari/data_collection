#!/usr/bin/env python3
"""Launch full system: data_collection_bringup + NJC controller nodes.

Single-command launch for the complete NJC pipeline.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
    bringup_share = get_package_share_directory('data_collection_bringup')
    njc_share = get_package_share_directory('njc_ros')

    model_dir_arg = DeclareLaunchArgument(
        'model_dir',
        description='Path to the NJC model directory (must contain config.yaml + checkpoint)',
    )

    data_collection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(bringup_share, 'launch', 'data_collection_bringup.launch.py')
        ),
    )

    njc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(njc_share, 'launch', 'njc_controller.launch.py')
        ),
        launch_arguments={'model_dir': LaunchConfiguration('model_dir')}.items(),
    )

    return LaunchDescription([
        model_dir_arg,
        data_collection_launch,
        njc_launch,
    ])
