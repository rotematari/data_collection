import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
    
 
    
    
def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="kinova_gen3_7",
            package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config",
        )
        .robot_description(
            file_path="/home/rotem/data_collection/src/ros2_kortex_jazzy/src/ros2_kortex/kortex_description/robots/gen3.xacro"
        )
        # Use MoveIt Simple Controller Manager (controllers YAML is a ROS param file)
        .trajectory_execution(
            file_path="/home/rotem/data_collection/src/moveit_pkg/config/moveit_controllers.yaml"
            )
        
        # Feed kinematics.yaml as robot_description_kinematics
        .robot_description_kinematics(
            file_path="/home/rotem/data_collection/src/moveit_pkg/config/kinematics.yaml"
        )
        # OMPL pipeline (the default). If you have a custom OMPL yaml, you can point to it:
        .planning_pipelines("ompl")
        .to_moveit_configs()
    )
    # launch_arguments = {
    #     "gripper": "robotiq_2f_85",
    #     "gripper_joint_name": "robotiq_85_left_knuckle_joint",
    #     "dof": "7",
    #     # Avoid passing robot_ip here; it's not used by the description xacro.
    #     # "use_fake_hardware": "false",  # only if your xacro supports it
    # }

    # cfg_pkg = "kinova_gen3_7dof_robotiq_2f_85_moveit_config"
    # controllers_yaml = os.path.join(
    #     get_package_share_directory(cfg_pkg), "config", "moveit_controllers.yaml"
    # )

    # moveit_config = (
    #     MoveItConfigsBuilder("gen3", package_name=cfg_pkg)
    #     .robot_description(mappings=launch_arguments)      # use the package’s default xacro + mappings
    #     .trajectory_execution(file_path=controllers_yaml)  # ROS-param formatted
    #     .planning_scene_monitor(publish_robot_description=True,
    #                             publish_robot_description_semantic=True)
    #     .planning_pipelines("ompl")                        # keep it simple first
    #     .to_moveit_configs()
    # )

    node = Node(
        package="moveit_pkg",
        executable="moveit_pose_goal_node",
        name="moveit_pose_goal_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
                    {
            # Make the pipeline unambiguously present:
            "planning_pipelines": {
                "pipeline_names": ["ompl"],
                "ompl": {
                    "planning_plugin": "ompl_interface/OMPLPlanner",
                    # standard adapters (one per line or space-separated):
                    "request_adapters": (
                        "default_planner_request_adapters/FixWorkspaceBounds "
                        "default_planner_request_adapters/FixStartStateBounds "
                        "default_planner_request_adapters/FixStartStateCollision "
                        "default_planner_request_adapters/FixStartStatePathConstraints"
                    ),
                    "start_state_max_bounds_error": 0.1,
                },
            },
        },# inject robot_description, SRDF, kinematics, pipelines
            {"controllers": ["joint_trajectory_controller"]},  # matches your ros2_control controller name
        ],
    )

    # moveit_config.moveit_cpp.update({"use_sim_time": use_sim_time.perform(context) == "true"})

    moveit_node = Node(
        package="moveit_pkg",
        executable="moveit_pose_goal_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),  # <-- ONLY this (plus any simple dicts you add)
            {"controllers": ["joint_trajectory_controller"]},
        ],
    )
    moveit_node = Node(
        package="moveit_pkg",
        executable="moveit_pose_goal_node",
        output="screen",
        parameters=[
                    moveit_config.to_dict(),],
    )

    return LaunchDescription(
        [
            moveit_node
        ]
    )