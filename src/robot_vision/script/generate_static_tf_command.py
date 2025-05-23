#!/usr/bin/env python3

import argparse
import rclpy
from rclpy.node import Node as RclpyNode # Alias to avoid confusion
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import sys
import time
import threading

# Helper function to create a 4x4 homogeneous transformation matrix
def create_transform_matrix(translation, quaternion_xyzw):
    mat = np.identity(4)
    mat[:3, :3] = Rotation.from_quat(quaternion_xyzw).as_matrix()
    mat[:3, 3] = translation
    return mat

# A simple class to handle TF listening within a temporary node
class TFListenerHelperNode(RclpyNode):
    def __init__(self, node_name='tf_listener_helper_node'):
        super().__init__(node_name)
        self.tf_buffer = Buffer()
        # Run the listener in a separate thread so it doesn't block the main script
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True) 
        self.get_logger().info(f"TFListenerHelperNode \'{node_name}\' initialized.")

    def lookup_transform_blocking(self, target_frame, source_frame, timeout_sec):
        self.get_logger().info(f"Attempting to look up transform from \'{source_frame}\' to \'{target_frame}\'.")
        try:
            # Use rclpy.time.Time() for now, or pass a specific time if needed
            transform_stamped = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(), # Get the latest available transform
                timeout=rclpy.duration.Duration(seconds=timeout_sec)
            )
            self.get_logger().info(f"Successfully looked up transform from \'{source_frame}\' to \'{target_frame}\'.")
            return transform_stamped
        except TransformException as ex:
            self.get_logger().warn(f"Could not get transform from \'{source_frame}\' to \'{target_frame}\': {ex}")
            return None
        except Exception as e:
            self.get_logger().error(f"Unexpected error during TF lookup: {e}")
            return None

    def shutdown(self):
        self.get_logger().info("Shutting down TFListenerHelperNode.")
        # The TransformListener's spin_thread option should handle its own thread shutdown.
        # Explicitly unregistering or stopping might be needed if issues arise,
        # but typically destroy_node is sufficient for resources managed by the node.
        if hasattr(self, 'tf_listener') and self.tf_listener:
            # Attempt to stop the listener thread explicitly if possible, though often not directly exposed
            # For now, relying on destroy_node and rclpy.shutdown()
            pass
        if rclpy.ok():
            self.destroy_node()


def main_logic(args):
    rclpy.init()
    tf_listener_node = TFListenerHelperNode()
    
    # Give TF listener a moment to connect and populate
    print("Initializing TF listener...")
    time.sleep(2.5) # Increased sleep to allow TF buffer to populate

    # T_World_BaseLink (fixed, provided by user)
    # Ensure quaternion is [x,y,z,w] for Scipy
    T_world_base_mat = create_transform_matrix(args.fixed_world_to_base_link_translation, args.fixed_world_to_base_link_quaternion)
    # T_BaseLink_World = inv(T_World_BaseLink)
    T_base_world_mat = np.linalg.inv(T_world_base_mat)

    collected_translations_base_cam = []
    collected_quaternions_base_cam_xyzw = []

    print(f"Starting to collect {args.num_samples} samples...")
    print(f"Listening for transform: TARGET=\'{args.world_frame_id}\' <- SOURCE=\'{args.externally_calibrated_camera_frame_id}\' (from your external calibration).")
    print(f"Using fixed transform \'{args.world_frame_id}\' -> \'{args.base_link_frame_id}\': t={args.fixed_world_to_base_link_translation}, q={args.fixed_world_to_base_link_quaternion}.")
    print(f"Will calculate and average \'{args.base_link_frame_id}\' -> \'{args.externally_calibrated_camera_frame_id}\'.")
    print(f"Output command will be for: '{args.output_parent_frame_id}' -> '{args.output_child_frame_id}'.")

    samples_collected = 0
    retries = 0
    max_retries = 3 # Max retries for a single sample if lookup fails initially

    while samples_collected < args.num_samples:
        if not rclpy.ok():
            print("RCLPY was shut down, exiting sample collection.")
            break
        
        # Lookup T_World_ExternalCam (transform from world to the externally calibrated camera)
        # Note: lookup_transform expects target_frame, source_frame
        # We want the pose of \'externally_calibrated_camera_frame_id\' (source) expressed in \'world_frame_id\' (target)
        transform_world_extcam_msg = tf_listener_node.lookup_transform_blocking(
            args.world_frame_id,                      # Target frame
            args.externally_calibrated_camera_frame_id, # Source frame
            args.lookup_timeout_sec
        )

        if transform_world_extcam_msg:
            t_w_extcam = transform_world_extcam_msg.transform.translation
            q_w_extcam = transform_world_extcam_msg.transform.rotation
            
            T_world_extcam_mat = create_transform_matrix(
                [t_w_extcam.x, t_w_extcam.y, t_w_extcam.z],
                [q_w_extcam.x, q_w_extcam.y, q_w_extcam.z, q_w_extcam.w] # ROS uses x,y,z,w
            )

            # Calculate T_BaseLink_ExternalCam = T_BaseLink_World * T_World_ExternalCam
            T_base_extcam_mat = T_base_world_mat @ T_world_extcam_mat

            # Extract translation and quaternion
            t_base_extcam = T_base_extcam_mat[:3, 3]
            q_base_extcam_xyzw = Rotation.from_matrix(T_base_extcam_mat[:3, :3]).as_quat() # Scipy gives [x,y,z,w]

            collected_translations_base_cam.append(t_base_extcam)
            collected_quaternions_base_cam_xyzw.append(q_base_extcam_xyzw)
            samples_collected += 1
            print(f"Sample {samples_collected}/{args.num_samples} (base_link -> cal_cam) collected.")
            retries = 0 # Reset retries on successful sample
        else:
            retries +=1
            print(f"Failed to collect sample (attempt {retries}/{max_retries}). Retrying after {args.sample_interval_sec}s...")
            if retries >= max_retries:
                print(f"Max retries reached for a sample. Check if the transform \'{args.world_frame_id}\' -> \'{args.externally_calibrated_camera_frame_id}\' is being published correctly.")
                # Optionally, decide if to break or continue trying for next samples
                # For now, we\'ll just wait and try for the next sample period.
                pass # Fall through to the sleep

        if samples_collected < args.num_samples:
            try:
                time.sleep(args.sample_interval_sec)
            except KeyboardInterrupt:
                print("\\nKeyboard interrupt detected during sleep. Exiting...")
                break


    if not collected_translations_base_cam or not collected_quaternions_base_cam_xyzw:
        print("No samples collected. Cannot generate command.")
    else:
        # Average translations
        avg_translation_base_cam = np.mean(collected_translations_base_cam, axis=0)
        # Average quaternions
        rotations_to_average = Rotation.from_quat(np.array(collected_quaternions_base_cam_xyzw)) # Expects [x,y,z,w]
        avg_rotation_base_cam = Rotation.mean(rotations_to_average)
        avg_quaternion_base_cam_xyzw = avg_rotation_base_cam.as_quat() # Returns [x,y,z,w]

        print(f"\nAveraged T_{args.output_parent_frame_id}_to_{args.output_child_frame_id}: "
              f"Translation (x,y,z): {[float(f'{v:.6f}') for v in avg_translation_base_cam]}, "
              f"Quaternion (x,y,z,w): {[float(f'{v:.6f}') for v in avg_quaternion_base_cam_xyzw]}")

        cmd = (
            f"python3 /path/to/your/static_transform_publisher_script.py \\\\\n" # Placeholder
            f"  --x {avg_translation_base_cam[0]:.6f} \\\\\n"
            f"  --y {avg_translation_base_cam[1]:.6f} \\\\\n"
            f"  --z {avg_translation_base_cam[2]:.6f} \\\\\n"
            f"  --qx {avg_quaternion_base_cam_xyzw[0]:.6f} \\\\\n"
            f"  --qy {avg_quaternion_base_cam_xyzw[1]:.6f} \\\\\n"
            f"  --qz {avg_quaternion_base_cam_xyzw[2]:.6f} \\\\\n"
            f"  --qw {avg_quaternion_base_cam_xyzw[3]:.6f} \\\\\n"
            f"  --frame-id {args.output_parent_frame_id} \\\\\n"
            f"  --child-frame-id {args.output_child_frame_id}"\
        )
        
        ros2_cmd = (
            f"ros2 run tf2_ros static_transform_publisher \\\\\n"
            f"  --x {avg_translation_base_cam[0]:.6f} \\\\\n"
            f"  --y {avg_translation_base_cam[1]:.6f} \\\\\n"
            f"  --z {avg_translation_base_cam[2]:.6f} \\\\\n"
            f"  --qx {avg_quaternion_base_cam_xyzw[0]:.6f} \\\\\n"
            f"  --qy {avg_quaternion_base_cam_xyzw[1]:.6f} \\\\\n"
            f"  --qz {avg_quaternion_base_cam_xyzw[2]:.6f} \\\\\n"
            f"  --qw {avg_quaternion_base_cam_xyzw[3]:.6f} \\\\\n"
            f"  --frame-id {args.output_parent_frame_id} \\\\\n"
            f"  --child-frame-id {args.output_child_frame_id}"\
        )

        print("\n" + "="*70 + "\n" +\
              "IF YOU HAVE A PYTHON SCRIPT TO PUBLISH A STATIC TRANSFORM, UPDATE IT WITH:" + "\n" +\
              "="*70 + "\n" +\
              f"Translation (x,y,z): {[float(f'{v:.6f}') for v in avg_translation_base_cam]}"+\
              f"Quaternion (x,y,z,w): {[float(f'{v:.6f}') for v in avg_quaternion_base_cam_xyzw]}"+\
              f"Parent Frame: {args.output_parent_frame_id}"+\
              f"Child Frame: {args.output_child_frame_id}"+\
              "="*70)
              
        print("\n" + "="*70 + "\n" +\
              "OR, USE THE FOLLOWING ros2 run COMMAND:" + "\n" +\
              "="*70 + "\n" +\
              ros2_cmd + "\n" +\
              "="*70)

    if rclpy.ok():
        tf_listener_node.shutdown() # Ensure node is shut down before rclpy.shutdown()
    if rclpy.ok(): # Check again as shutdown might take a moment or be called elsewhere
        rclpy.shutdown()
    print("Script finished.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Listens to an externally published camera TF transform (world->camera), "\
                    "calculates its pose relative to a fixed base_link (base_link->camera), "\
                    "averages it, and generates a static_transform_publisher command or values for a Python script.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument('--world-frame-id', type=str, default='world', help='Frame ID of the world. Default: world')
    parser.add_argument('--base-link-frame-id', type=str, default='base_link', help='Frame ID of the robot base_link. Default: base_link')
    parser.add_argument('--externally-calibrated-camera-frame-id', type=str, required=True, help='Frame ID of the camera being published by your external calibration (relative to world). E.g., camera_link_calibrated')
    
    parser.add_argument('--fixed-world-to-base-link-translation', type=float, nargs=3, required=True, metavar=('X', 'Y', 'Z'), help='KNOWN Translation [x y z] of base_link in world frame. E.g., 0.0 0.0 0.0')
    parser.add_argument('--fixed-world-to-base-link-quaternion', type=float, nargs=4, required=True, metavar=('QX', 'QY', 'QZ', 'QW'), help='KNOWN Quaternion [x y z w] of base_link in world frame. E.g., 0.0 0.0 0.0 1.0')
    
    parser.add_argument('--num-samples', type=int, default=10, help='Number of transform samples to average. Default: 10')
    parser.add_argument('--sample-interval-sec', type=float, default=1.0, help='Interval in seconds between TF lookup attempts. Default: 1.0')
    parser.add_argument('--lookup-timeout-sec', type=float, default=2.0, help='Timeout for each TF lookup. Default: 2.0')
    
    parser.add_argument('--output-parent-frame-id', type=str, default='base_link', help='Desired parent frame for the output static_transform_publisher command (usually base_link). Default: base_link')
    parser.add_argument('--output-child-frame-id', type=str, required=True, help='Desired child frame for the output static_transform_publisher command (e.g., default_cam_averaged).')

    if len(sys.argv) <= 1: # If only script name is called (or no args)
        parser.print_help(sys.stderr)
        sys.exit(1)
        
    args = parser.parse_args()

    try:
        main_logic(args)
    except KeyboardInterrupt:
        print("\\nKeyboardInterrupt detected in main. Shutting down...")
    finally:
        if rclpy.ok():
            rclpy.shutdown() # Ensure rclpy is shutdown in case of error in main_logic before its own shutdown call
        print("Exiting script.")
