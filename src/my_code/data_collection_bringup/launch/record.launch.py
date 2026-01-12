from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import time
import launch
from launch.actions import TimerAction, Shutdown
def launch_setup(context, *args, **kwargs):
    name_value = LaunchConfiguration("name").perform(context)
    timestamp = time.strftime("%m_%d_%H_%M")
    directory = LaunchConfiguration("directory").perform(context)
    bag_name = f"{directory}{timestamp}_{name_value}"
    duration = LaunchConfiguration("duration").perform(context)
    shutdown_timer = TimerAction(
            period=float(duration),
            actions=[
                Shutdown(reason='Recording duration reached')
            ]
        )
    
    
    throttle_topics = [
        "/digit/D21118/image_raw",
        "/digit/D21122/image_raw",
        "/digit/D21123/image_raw",
        "/digit/D21124/image_raw",
        "/joint_states",
        "/natnet/unlabeled_marker_data",
        "/natnet/fixed_ee_pose",
        "/end_effector_pose",
    ]
    throttle_nodes = []
    for topic in throttle_topics:
        topic_key = topic.strip("/").replace("/", "_")
        throttle_nodes.append(
            Node(
                package="topic_tools",
                executable="throttle",
                name=f"throttle_{topic_key}",
                arguments=[
                    "messages",
                    topic,
                    LaunchConfiguration("rate_hz"),
                    f"{topic}_throttled",
                ],
                output="screen",
            )
        )
    recorder_node =Node(
                package="rosbag2_transport",
                executable="recorder",
                name="recorder",
                output="screen",
                parameters=[
                    "/home/rotem/data_collection/src/my_code/data_collection_bringup/config/record.yaml",
                    {"storage.uri": bag_name},
                ],
            )
    return [
        *throttle_nodes,
        recorder_node,
        shutdown_timer,
    ]

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "name",
            default_value="",
            description="Name of the rosbag to record to",
        ),
        DeclareLaunchArgument(
            "directory",
            default_value="/home/rotem/data_collection/recordings/",
            description="Directory to save the rosbag recordings",
        ),
        DeclareLaunchArgument(
            "duration",
            default_value="600.0",
            description="Duration to record the rosbag in seconds",
        ),
        DeclareLaunchArgument(
        'rate_hz',
        default_value='10.0',
        description='Recording rate in Hz'
        ),     
        
    ]



    return launch.LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup),

    ])
