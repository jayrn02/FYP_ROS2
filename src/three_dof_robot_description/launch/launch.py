import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Get the path to the URDF file
    urdf_file_name = 'threeDOFrobot.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('three_dof_robot_description'),
        'urdf',
        urdf_file_name)
    
    # Read the URDF contents
    with open(urdf_path, 'r') as file:
        robot_description_content = file.read()

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Node: robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    # Node: joint_state_publisher_gui
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'))

    # Add nodes to the launch description
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)

    return ld