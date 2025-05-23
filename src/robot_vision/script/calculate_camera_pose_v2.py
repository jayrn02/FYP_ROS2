#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import numpy as np
from scipy.spatial.transform import Rotation

import tf2_ros
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import StaticTransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import sys
import time

def create_transform_matrix(translation, quaternion_xyzw):
    """Creates a 4x4 homogeneous transformation matrix from translation and [x,y,z,w] quaternion."""
    mat = np.identity(4)
    mat[:3, :3] = Rotation.from_quat(quaternion_xyzw).as_matrix()
    mat[:3, 3] = translation
    return mat

class AutoCameraCalibrator(Node):
    def __init__(self):
        super().__init__('auto_camera_calibrator_node')

        # Declare parameters
        self.declare_parameter('marker_world_translation', [0.0, 0.0, 0.0],
                               ParameterDescriptor(description='Translation [x, y, z] of the marker in the world frame (T_Marker_World).', type=ParameterType.PARAMETER_DOUBLE_ARRAY))
        self.declare_parameter('marker_world_quaternion', [0.0, 0.0, 0.0, 1.0],
                               ParameterDescriptor(description='Rotation Quaternion [x, y, z, w] of the marker in the world frame (T_Marker_World).', type=ParameterType.PARAMETER_DOUBLE_ARRAY))
        
        # New parameters for the known robot base pose in the world
        self.declare_parameter('robot_base_world_translation', [0.0, 0.0, 0.0],
                               ParameterDescriptor(description='KNOWN Translation [x, y, z] of the robot base_link in the world frame (T_BaseLink_World).', type=ParameterType.PARAMETER_DOUBLE_ARRAY))
        self.declare_parameter('robot_base_world_quaternion', [0.0, 0.0, 0.0, 1.0],
                               ParameterDescriptor(description='KNOWN Rotation Quaternion [x, y, z, w] of the robot base_link in the world frame (T_BaseLink_World).', type=ParameterType.PARAMETER_DOUBLE_ARRAY))

        self.declare_parameter('num_samples', 10,
                               ParameterDescriptor(description='Number of transform samples to average.', type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('camera_optical_frame_for_lookup', 'default_cam', 
                               ParameterDescriptor(description='TF frame ID of the camera\\'s optical sensor used for TF lookup (e.g., default_cam, camera_color_optical_frame). This is the TARGET frame when looking up marker pose.', type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('aruco_marker_tf_frame', 'aruco_marker_frame',
                               ParameterDescriptor(description='TF frame ID of the ArUco marker being detected (SOURCE frame for T_Marker_Camera lookup).', type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('world_frame', 'world', # Used for T_Marker_World and T_BaseLink_World
                               ParameterDescriptor(description='TF frame ID of the fixed world.', type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('base_link_frame', 'base_link', # Parent frame for the output transform
                               ParameterDescriptor(description='TF frame ID of the robot base (parent for the calibrated camera transform).', type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('calibrated_camera_frame_name', 'default_cam_calibrated', # Child frame for the output transform
                               ParameterDescriptor(description='Desired TF frame ID for the calibrated camera pose relative to base_link.', type=ParameterType.PARAMETER_STRING))
        
        self.declare_parameter('lookup_timeout_sec', 2.0,
                                ParameterDescriptor(description='Timeout in seconds for each TF lookup attempt.', type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('sample_interval_sec', 0.5,
                                ParameterDescriptor(description='Interval in seconds between TF lookup attempts.', type=ParameterType.PARAMETER_DOUBLE))

        # Get parameters
        self.t_marker_world = np.array(self.get_parameter('marker_world_translation').value)
        self.q_marker_world = np.array(self.get_parameter('marker_world_quaternion').value)
        self.t_base_world = np.array(self.get_parameter('robot_base_world_translation').value)
        self.q_base_world = np.array(self.get_parameter('robot_base_world_quaternion').value)

        self.num_samples = self.get_parameter('num_samples').value
        self.camera_optical_frame_for_lookup = self.get_parameter('camera_optical_frame_for_lookup').value
        self.aruco_marker_tf_frame = self.get_parameter('aruco_marker_tf_frame').value
        self.world_frame_param = self.get_parameter('world_frame').value # To confirm consistency, not directly used in matrix names
        self.base_link_frame_param = self.get_parameter('base_link_frame').value
        self.calibrated_camera_frame_name = self.get_parameter('calibrated_camera_frame_name').value
        self.lookup_timeout_sec = self.get_parameter('lookup_timeout_sec').value
        self.sample_interval_sec = self.get_parameter('sample_interval_sec').value

        if not (len(self.t_marker_world) == 3 and len(self.q_marker_world) == 4):
            self.get_logger().error("marker_world_translation must be 3 elements and marker_world_quaternion must be 4 elements.")
            sys.exit(1)
        if not (len(self.t_base_world) == 3 and len(self.q_base_world) == 4):
            self.get_logger().error("robot_base_world_translation must be 3 elements and robot_base_world_quaternion must be 4 elements.")
            sys.exit(1)

        self.get_logger().info(f"Objective: Calculate T_BaseLink_CameraOpticalFrame")
        self.get_logger().info(f"KNOWN T_Marker_World: t={self.t_marker_world}, q={self.q_marker_world} (frame: {self.world_frame_param})")
        self.get_logger().info(f"KNOWN T_BaseLink_World: t={self.t_base_world}, q={self.q_base_world} (frame: {self.world_frame_param} to {self.base_link_frame_param})")
        self.get_logger().info(f"Number of samples to average: {self.num_samples}")
        self.get_logger().info(f"Looking for transform from '{self.camera_optical_frame_for_lookup}' (target) to '{self.aruco_marker_tf_frame}' (source) to find T_CameraOptical_Marker")
        self.get_logger().info(f"Output static transform: '{self.base_link_frame_param}' -> '{self.calibrated_camera_frame_name}'")


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.collected_translations_cam_marker = []
        self.collected_quaternions_cam_marker_xyzw = [] # Storing as [x,y,z,w]

        self.sample_collection_timer = self.create_timer(self.sample_interval_sec, self.collect_sample)
        self.get_logger().info("Auto camera calibrator node started. Collecting samples...")

    def collect_sample(self):
        if len(self.collected_translations_cam_marker) >= self.num_samples:
            self.get_logger().info(f"Collected {len(self.collected_translations_cam_marker)} samples. Proceeding to calculation.")
            self.sample_collection_timer.cancel() # Stop collecting
            self.process_and_publish()
            return

        try:
            # We want the transform that expresses the marker's frame in the camera's optical frame coordinates.
            # lookup_transform(target_frame, source_frame, ...)
            # T_CameraOptical_Marker: target=camera_optical_frame_for_lookup, source=aruco_marker_tf_frame
            now = rclpy.time.Time()
            trans_msg = self.tf_buffer.lookup_transform(
                self.camera_optical_frame_for_lookup, # Target frame
                self.aruco_marker_tf_frame,          # Source frame
                now,
                timeout=rclpy.duration.Duration(seconds=self.lookup_timeout_sec))

            t = trans_msg.transform.translation
            q = trans_msg.transform.rotation
            self.collected_translations_cam_marker.append(np.array([t.x, t.y, t.z]))
            self.collected_quaternions_cam_marker_xyzw.append(np.array([q.x, q.y, q.z, q.w])) #x,y,z,w
            self.get_logger().info(f"Sample {len(self.collected_translations_cam_marker)}/{self.num_samples} (T_Camera_Marker) collected.")

        except TransformException as ex:
            self.get_logger().warn(f"Could not get transform {self.camera_optical_frame_for_lookup} -> {self.aruco_marker_tf_frame}: {ex}. Retrying...")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred during sample collection: {e}")


    def process_and_publish(self):
        if not self.collected_translations_cam_marker or not self.collected_quaternions_cam_marker_xyzw:
            self.get_logger().error("No samples collected. Cannot proceed.")
            rclpy.shutdown()
            return

        # Average translations for T_Camera_Marker
        avg_translation_cam_marker = np.mean(self.collected_translations_cam_marker, axis=0)

        # Average quaternions for T_Camera_Marker using Scipy
        rotations_to_average_cam_marker = Rotation.from_quat(np.array(self.collected_quaternions_cam_marker_xyzw))
        avg_rotation_cam_marker = Rotation.mean(rotations_to_average_cam_marker)
        avg_quaternion_cam_marker_xyzw = avg_rotation_cam_marker.as_quat() # [x,y,z,w]

        self.get_logger().info(f"Averaged T_CameraOptical_Marker: t={avg_translation_cam_marker}, q={avg_quaternion_cam_marker_xyzw}")

        # This is T_CameraOptical_Marker_avg (e.g., T_DefaultCam_Marker_avg)
        T_cam_marker_avg_mat = create_transform_matrix(avg_translation_cam_marker, avg_quaternion_cam_marker_xyzw)

        # This is T_Marker_World (from parameters)
        T_marker_world_mat = create_transform_matrix(self.t_marker_world, self.q_marker_world)
        
        # This is T_BaseLink_World (from parameters, inverse of T_World_BaseLink)
        T_world_base_mat = create_transform_matrix(self.t_base_world, self.q_base_world) # This is T_World_BaseLink
        T_base_world_mat = np.linalg.inv(T_world_base_mat) # This is T_BaseLink_World

        # Calculate T_BaseLink_CameraOpticalFrame = T_BaseLink_World * T_World_Marker * inv(T_CameraOptical_Marker_avg)
        # inv(T_CameraOptical_Marker_avg) is T_Marker_CameraOptical_avg
        
        T_base_cam_final_mat = T_base_world_mat @ T_marker_world_mat @ np.linalg.inv(T_cam_marker_avg_mat)
        
        # Extract final translation and quaternion for T_BaseLink_CameraOpticalFrame
        t_base_cam = T_base_cam_final_mat[:3, 3]
        q_base_cam_xyzw = Rotation.from_matrix(T_base_cam_final_mat[:3, :3]).as_quat()

        self.get_logger().info(f"Calculated T_BaseLink_CameraOpticalFrame ('{self.base_link_frame_param}' -> '{self.calibrated_camera_frame_name}'): t={list(t_base_cam)}, q={list(q_base_cam_xyzw)}")
        self.get_logger().info("You can use these values to update your static_transform_publisher for the camera on the robot.")

        # Publish the static transform for BaseLink -> CalibratedCameraFrameName
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = self.base_link_frame_param # Parent frame
        static_transform_stamped.child_frame_id = self.calibrated_camera_frame_name # Child frame
        static_transform_stamped.transform.translation.x = t_base_cam[0]
        static_transform_stamped.transform.translation.y = t_base_cam[1]
        static_transform_stamped.transform.translation.z = t_base_cam[2]
        static_transform_stamped.transform.rotation.x = q_base_cam_xyzw[0]
        static_transform_stamped.transform.rotation.y = q_base_cam_xyzw[1]
        static_transform_stamped.transform.rotation.z = q_base_cam_xyzw[2]
        static_transform_stamped.transform.rotation.w = q_base_cam_xyzw[3]

        self.static_broadcaster.sendTransform(static_transform_stamped)
        self.get_logger().info(f"Successfully published static transform: '{self.base_link_frame_param}' -> '{self.calibrated_camera_frame_name}'. Node will now shut down.")
        
        # Shutdown after publishing
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = None # Initialize node to None
    try:
        node = AutoCameraCalibrator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("KeyboardInterrupt, shutting down.")
    except Exception as e:
        if node and rclpy.ok(): 
            node.get_logger().fatal(f"Unhandled exception in main: {e}")
        else:
            print(f"Unhandled exception in main (rclpy context invalid or node not initialized): {e}", file=sys.stderr)
    finally:
        if node and rclpy.ok() and node.executor: # Check if node exists, rclpy is ok and node has an executor
            node.destroy_node()
        if rclpy.ok():
             rclpy.try_shutdown()

if __name__ == '__main__':
    main()
