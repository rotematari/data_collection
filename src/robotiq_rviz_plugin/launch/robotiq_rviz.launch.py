from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('robotiq_rviz_plugin')
    rviz_config = os.path.join(package_dir, 'config', 'robotiq_rviz.rviz')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='/dev/ttyUSB0',
            description='Serial device for gripper'
        ),
        
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz'
        ),
        
        Node(
            package='robotiq_rviz_plugin',
            executable='robotiq_gripper_node.py',
            name='robotiq_gripper_node',
            parameters=[{
                'device': LaunchConfiguration('device'),
                'baudrate': 115200,
                # 'status_rate': 10.0
            }],
            output='screen'
        ),
        
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config],
        #     condition=IfCondition(
        #         LaunchConfiguration('rviz')
        #     )
        # )
    ])