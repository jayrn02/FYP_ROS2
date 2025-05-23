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
    # Ensure quaternion is in [x, y, z, w] order for SciPy
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
        self.declare_parameter('num_samples', 10,
                               ParameterDescriptor(description='Number of transform samples to average.', type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('camera_optical_frame', 'camera_color_optical_frame',
                               ParameterDescriptor(description='TF frame ID of the camera's optical sensor (parent frame for T_Marker_Camera lookup).', type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('aruco_marker_tf_frame', 'aruco_marker_frame', # Example, might be 'marker_0' or similar from aruco_opencv
                               ParameterDescriptor(description='TF frame ID of the ArUco marker being detected (child frame for T_Marker_Camera lookup).', type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('world_frame', 'world',
                               ParameterDescriptor(description='TF frame ID of the fixed world.', type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('calibrated_camera_frame_name', 'calibrated_camera_link',
                               ParameterDescriptor(description='Desired TF frame ID for the calibrated camera pose.', type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('lookup_timeout_sec', 2.0,
                                ParameterDescriptor(description='Timeout in seconds for each TF lookup attempt.', type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('sample_interval_sec', 0.5,
                                ParameterDescriptor(description='Interval in seconds between TF lookup attempts.', type=ParameterType.PARAMETER_DOUBLE))


        # Get parameters
        self.t_marker_world = np.array(self.get_parameter('marker_world_translation').value)
        self.q_marker_world = np.array(self.get_parameter('marker_world_quaternion').value)
        self.num_samples = self.get_parameter('num_samples').value
        self.camera_optical_frame = self.get_parameter('camera_optical_frame').value
        self.aruco_marker_tf_frame = self.get_parameter('aruco_marker_tf_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.calibrated_camera_frame_name = self.get_parameter('calibrated_camera_frame_name').value
        self.lookup_timeout_sec = self.get_parameter('lookup_timeout_sec').value
        self.sample_interval_sec = self.get_parameter('sample_interval_sec').value

        if not (len(self.t_marker_world) == 3 and len(self.q_marker_world) == 4):
            self.get_logger().error("marker_world_translation must be 3 elements and marker_world_quaternion must be 4 elements.")
            sys.exit(1)

        self.get_logger().info(f"Target T_Marker_World: t={self.t_marker_world}, q={self.q_marker_world}")
        self.get_logger().info(f"Number of samples to average: {self.num_samples}")
        self.get_logger().info(f"Looking for transform from '{self.camera_optical_frame}' (target) to '{self.aruco_marker_tf_frame}' (source)")
        self.get_logger().info(f"Output static transform: '{self.world_frame}' -> '{self.calibrated_camera_frame_name}'")


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.collected_translations = []
        self.collected_quaternions_xyzw = [] # Storing as [x,y,z,w]

        self.sample_collection_timer = self.create_timer(self.sample_interval_sec, self.collect_sample)
        self.get_logger().info("Auto camera calibrator node started. Collecting samples...")

    def collect_sample(self):
        if len(self.collected_translations) >= self.num_samples:
            self.get_logger().info(f"Collected {len(self.collected_translations)} samples. Proceeding to calculation.")
            self.sample_collection_timer.cancel() # Stop collecting
            self.process_and_publish()
            return

        try:
            # We want the transform that expresses the marker's frame in the camera's optical frame coordinates.
            # lookup_transform(target_frame, source_frame, ...)
            # T_Marker_Camera: target=camera_optical_frame, source=aruco_marker_tf_frame
            now = rclpy.time.Time()
            trans_msg = self.tf_buffer.lookup_transform(
                self.camera_optical_frame, # Target frame
                self.aruco_marker_tf_frame,  # Source frame
                now,
                timeout=rclpy.duration.Duration(seconds=self.lookup_timeout_sec))

            t = trans_msg.transform.translation
            q = trans_msg.transform.rotation
            self.collected_translations.append(np.array([t.x, t.y, t.z]))
            self.collected_quaternions_xyzw.append(np.array([q.x, q.y, q.z, q.w]))
            self.get_logger().info(f"Sample {len(self.collected_translations)}/{self.num_samples} collected.")

        except TransformException as ex:
            self.get_logger().warn(f"Could not get transform {self.camera_optical_frame} -> {self.aruco_marker_tf_frame}: {ex}. Retrying...")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred during sample collection: {e}")


    def process_and_publish(self):
        if not self.collected_translations or not self.collected_quaternions_xyzw:
            self.get_logger().error("No samples collected. Cannot proceed.")
            rclpy.shutdown()
            return

        # Average translations
        avg_translation = np.mean(self.collected_translations, axis=0)

        # Average quaternions using Scipy
        # Ensure all quaternions are in [x,y,z,w] format for Scipy
        rotations_to_average = Rotation.from_quat(np.array(self.collected_quaternions_xyzw))
        avg_rotation_object = Rotation.mean(rotations_to_average)
        avg_quaternion_xyzw = avg_rotation_object.as_quat() # [x,y,z,w]

        self.get_logger().info(f"Averaged T_Marker_Camera: t={avg_translation}, q={avg_quaternion_xyzw}")

        # This is T_Marker_Camera_avg
        T_marker_camera_avg_mat = create_transform_matrix(avg_translation, avg_quaternion_xyzw)

        # This is T_Marker_World (from parameters)
        T_marker_world_mat = create_transform_matrix(self.t_marker_world, self.q_marker_world)

        # Calculate T_Camera_World = T_Marker_World * inv(T_Marker_Camera_avg)
        T_camera_world_mat = T_marker_world_mat @ np.linalg.inv(T_marker_camera_avg_mat)

        # Extract final translation and quaternion for T_Camera_World
        t_camera_world = T_camera_world_mat[:3, 3]
        q_camera_world_xyzw = Rotation.from_matrix(T_camera_world_mat[:3, :3]).as_quat()

        self.get_logger().info(f"Calculated T_Camera_World: t={t_camera_world}, q={q_camera_world_xyzw}")

        # Publish the static transform
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = self.world_frame
        static_transform_stamped.child_frame_id = self.calibrated_camera_frame_name
        static_transform_stamped.transform.translation.x = t_camera_world[0]
        static_transform_stamped.transform.translation.y = t_camera_world[1]
        static_transform_stamped.transform.translation.z = t_camera_world[2]
        static_transform_stamped.transform.rotation.x = q_camera_world_xyzw[0]
        static_transform_stamped.transform.rotation.y = q_camera_world_xyzw[1]
        static_transform_stamped.transform.rotation.z = q_camera_world_xyzw[2]
        static_transform_stamped.transform.rotation.w = q_camera_world_xyzw[3]

        self.static_broadcaster.sendTransform(static_transform_stamped)
        self.get_logger().info(f"Successfully published static transform: '{self.world_frame}' -> '{self.calibrated_camera_frame_name}'. Node will now shut down.")
        
        # Shutdown after publishing
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = AutoCameraCalibrator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if rclpy.ok(): # Check if rclpy context is still valid
            node.get_logger().fatal(f"Unhandled exception in main: {e}")
    finally:
        if rclpy.ok() and 'node' in locals() and node: # Ensure node exists and rclpy is ok
            node.destroy_node()
        if rclpy.ok(): # Check if rclpy context is still valid before shutting down
             rclpy.try_shutdown() # Use try_shutdown to avoid error if already shutdown

if __name__ == '__main__':
    main()
