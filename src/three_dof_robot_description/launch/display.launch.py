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
    # Publishes TF transformations based on the URDF and joint states
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
    # Provides sliders to manually set joint states
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Node: rviz2
    # Visualization tool
    rviz_config = os.path.join(
        get_package_share_directory('three_dof_robot_description'),
        'rviz',
        'robot_display.rviz')
    
    # Check if the rviz config file exists
    use_default_rviz = not os.path.exists(rviz_config)
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config] if not use_default_rviz else []
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
    ld.add_action(rviz_node)

    return ld