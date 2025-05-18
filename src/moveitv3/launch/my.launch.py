import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo # Import LogInfo
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import yaml # Import yaml

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="moveit.rviz",
            description="RViz configuration file",
        )
    )

    # Initialize MoveItConfigsBuilder
    moveit_config = (
        MoveItConfigsBuilder("robot", package_name="moveitv3")
        .robot_description(file_path="config/robot.urdf.xacro")
        .robot_description_semantic(file_path="config/robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml") # This should load the controller params
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # --- Debugging: Print the generated MoveGroup parameters ---
    move_group_params_dict = moveit_config.to_dict().get('move_group', {})
    log_move_group_params = LogInfo(msg=f"MoveGroup Params: {yaml.dump(move_group_params_dict)}")
    # --- End Debugging ---


    # Get controller params (for ros2_control node)
    ros2_controllers_path = PathJoinSubstitution(
        [FindPackageShare("moveitv3"), "config", "ros2_controllers.yaml"]
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("moveitv3"), "config", LaunchConfiguration("rviz_config")]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # MoveGroup node - using the generated config
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        # Pass the generated parameters directly
        parameters=[moveit_config.to_dict()],
        # Add arguments for logging if needed
        # arguments=["--ros-args", "--log-level", "debug"],
    )


    # ros2_control node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    # --- Define Spawners ---
    # ... (spawner definitions remain the same) ...
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
            ],
        output="screen",
    )

    arm_controller_spawner = Node(
         package="controller_manager",
         executable="spawner",
         arguments=[
             "arm_controller",
             "--controller-manager", "/controller_manager",
             "--controller-manager-timeout", "30",
             ],
         output="screen",
    )

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "hand_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
            ],
        output="screen",
    )


    # --- Sequential Spawning ---
    # ... (event handlers remain the same) ...
    delay_jsb_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    delay_arm_spawner_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    delay_hand_spawner_after_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[hand_controller_spawner],
        )
    )


    # --- Nodes to start initially ---
    nodes_to_start = [
        log_move_group_params, # Add the logger action here
        robot_state_publisher_node,
        rviz_node,
        move_group_node,
        ros2_control_node,
        delay_jsb_spawner,
        delay_arm_spawner_after_jsb,
        delay_hand_spawner_after_arm,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)