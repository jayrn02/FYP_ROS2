import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_robot_vision = get_package_share_directory('robot_vision')
    camera_name = 'logitech_c270'
    camera_calibration_file = f'package://robot_vision/config/camera_calibration.yaml'

    # usb_cam node remains the same
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace='camera',
        output='screen',
        parameters=[
            {'video_device': '/dev/video0'},
            {'image_width': 640},
            {'image_height': 480},
            {'pixel_format': 'mjpeg2rgb'},
            {'camera_frame_id': 'default_cam'},
            {'camera_info_url': camera_calibration_file},
            {'camera_name': camera_name},
	    {'brightness': 160},
        ]
    )
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_camera_tf_publisher', # Descriptive name
        arguments=[
            # Use the new-style flags explicitly
            '--x', '-0.0009',
            '--y', '-0.1829',
            '--z', '0.8318',
            '--qx', '-0.6871',
            '--qy', '0.7220',
            '--qz', '-0.0314',
            '--qw', '0.0749',
            '--frame-id', 'base_link',         # IMPORTANT: Keep as base_link
            '--child-frame-id', 'default_cam'  # IMPORTANT: Keep as default_cam
        ]
    )

    # Define the image_proc component node WITH remappings
    image_proc_node = ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='image_proc',
            namespace='camera', # Still runs in the camera namespace
            # *** ADD THIS remappings SECTION ***
            remappings=[
                # Map the component's internal 'image' input to the absolute topic name
                ('image', '/camera/image_raw'),
                # Map the component's internal 'camera_info' input to the absolute topic name
                ('camera_info', '/camera/camera_info')
                # Add other remappings here if needed for output topics, but defaults should be okay
                # ('image_rect', '/camera/image_rect_explicit') # Example if you wanted to rename output
            ]
        )

    # Container definition remains the same
    image_proc_container = ComposableNodeContainer(
            name='image_proc_container',
            namespace='camera',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                image_proc_node,
            ],
            output='screen',
    )
    color_detector_node = Node(
        package='robot_vision',
        executable='color_detector.py', # The name of your python script
        name='color_detector_node',
        output='screen',
        # Override parameters here if needed, or do it at the other python script... 
        parameters=[
            # Hue range for green
            {'hsv_lower_hue1': 40},  # Lower bound for green hue
            {'hsv_upper_hue1': 80},  # Upper bound for green hue

            # Saturation range (adjust based on color purity)
            {'hsv_lower_sat': 70},   # Min saturation (avoid grays/whites)
            {'hsv_upper_sat': 255},  # Max saturation

            # Value range (adjust based on brightness)
            {'hsv_lower_val': 50},   # Min brightness (avoid blacks/shadows)
            {'hsv_upper_val': 255},  # Max brightness

            # We don't need the second hue range for green
            {'hsv_lower_hue2': 0},    # Not used for green
            {'hsv_upper_hue2': 0},    # Not used for green

            # Minimum contour area (adjust if needed)
            {'min_contour_area': 500.0}
        ]
    )
    # Add this Node definition inside generate_launch_description()

    pixel_to_3d_node = Node(
        package='robot_vision',
        executable='pixel_to_3d.py',
        name='pixel_to_3d_node',
        output='screen',
        parameters=[
            # Set the target frame where Z=0 is the workspace plane
            {'target_frame': 'base_link'}
        ]
    )

# Add it to the list of nodes returned by LaunchDescription
    return LaunchDescription([
        usb_cam_node,
        image_proc_container,
        static_tf_publisher,
        color_detector_node,
        pixel_to_3d_node # Add the new node here
    ])


    # Return usb_cam and the container
    return LaunchDescription([
        usb_cam_node,
        image_proc_container,
	static_tf_publisher,
	color_detector_node
    ])
