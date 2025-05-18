import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    log_level = LaunchConfiguration("log_level")

    # MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("robot", package_name="moveitv3")
        .robot_description(
            file_path="config/robot.urdf.xacro",
            # mappings={ # Add mappings if needed, e.g., for initial positions
            #     "initial_positions_file": PathJoinSubstitution(
            #         [FindPackageShare("moveitv3"), "config", "initial_positions.yaml"]
            #     )
            # }
        )
        .robot_description_semantic(file_path="config/robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"] # Add/remove pipelines as needed
        )
        .to_moveit_configs()
    )

    # Get paths
    moveitv3_share = FindPackageShare("moveitv3").perform(context)

    # --- ROS2 Control Node ---
    # Load hardware parameters
    hardware_config_path = PathJoinSubstitution(
        [moveitv3_share, "config", "hardware_config.yaml"]
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager", # <<< Add this line to explicitly name the node
        parameters=[
            hardware_config_path
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    # --- Controller Spawners ---
    # Load controllers defined in ros2_controllers.yaml
    controllers_yaml_path = PathJoinSubstitution(
        [moveitv3_share, "config", "ros2_controllers.yaml"]
    )

    # Ensure controllers_yaml_path is passed to controller_manager if needed,
    # but typically spawners load controllers listed in their args.

    # Spawn controllers after ros2_control_node is active
    # Delay start of spawners using event handlers or manually after launch if needed

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--ros-args", "--log-level", log_level],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager", "--ros-args", "--log-level", log_level],
        output="screen",
    )

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_controller", "--controller-manager", "/controller_manager", "--ros-args", "--log-level", log_level],
        output="screen",
    )

    # --- MoveGroup Node ---
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
             # Add any extra move_group params here
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # --- RViz ---
    rviz_config = PathJoinSubstitution(
        [moveitv3_share, "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config, "--ros-args", "--log-level", log_level],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_rviz),
    )

    # --- Static TF ---
    # Publishes tf for links defined in URDF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time}
        ],
    )

    nodes_to_start = [
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        hand_controller_spawner,
        move_group_node,
        rviz_node,
    ]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false", # Use false for real hardware
            description="Use simulation (Gazebo) clock if true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz", default_value="true", description="Launch RViz?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level", default_value="info", description="Log level (debug, info, warn, error, fatal)"
        )
    )


    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
