#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation
import argparse
import sys

def create_transform_matrix(translation, quaternion):
    """Creates a 4x4 homogeneous transformation matrix from translation and quaternion."""
    mat = np.identity(4)
    mat[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    mat[:3, 3] = translation
    return mat

def main():
    parser = argparse.ArgumentParser(description='Calculate the camera pose in the world frame (T_Camera_World).')

    # --- T_Marker_World Arguments ---
    parser.add_argument('--t_mw', type=float, nargs=3, required=True,
                        metavar=('TX', 'TY', 'TZ'),
                        help='Translation [x y z] of the marker in the world frame (T_Marker_World). Example: 0.346 0.0 -0.02')
    parser.add_argument('--q_mw', type=float, nargs=4, required=True,
                        metavar=('QX', 'QY', 'QZ', 'QW'),
                        help='Rotation Quaternion [x y z w] of the marker in the world frame (T_Marker_World). Example for no rotation: 0.0 0.0 0.0 1.0')

    # --- T_Marker_Camera Arguments ---
    parser.add_argument('--t_mc', type=float, nargs=3, required=True,
                        metavar=('TX', 'TY', 'TZ'),
                        help='Translation [x y z] of the marker relative to the camera frame (T_Marker_Camera), from tf2_echo.')
    parser.add_argument('--q_mc', type=float, nargs=4, required=True,
                        metavar=('QX', 'QY', 'QZ', 'QW'),
                        help='Rotation Quaternion [x y z w] of the marker relative to the camera frame (T_Marker_Camera), from tf2_echo.')

    # --- Frame Names (Optional, for output clarity) ---
    parser.add_argument('--world_frame', type=str, default='world',
                        help='Name of the world frame (e.g., base_link, map). Default: world')
    parser.add_argument('--camera_frame', type=str, default='camera_link',
                        help='Name of the camera frame you want to define (e.g., default_cam, camera_link_optical). Default: camera_link')


    # --- Handle potential "--help" or errors ---
    # If no arguments are given, print help and exit.
    if len(sys.argv) == 1:
        parser.print_help(sys.stderr)
        sys.exit(1)

    args = parser.parse_args()

    # --- Input Data ---
    # T_Marker_World: Transform from World frame TO Marker frame
    t_marker_world = np.array(args.t_mw)
    q_marker_world = np.array(args.q_mw) # Must be [x, y, z, w]
    T_marker_world_mat = create_transform_matrix(t_marker_world, q_marker_world)

    # T_Marker_Camera: Transform from Camera frame TO Marker frame
    t_marker_camera = np.array(args.t_mc)
    q_marker_camera = np.array(args.q_mc) # Must be [x, y, z, w]
    T_marker_camera_mat = create_transform_matrix(t_marker_camera, q_marker_camera)

    # --- Calculation ---
    # We want T_Camera_World: Transform from World frame TO Camera frame
    # Relationship: T_Camera_World = T_Marker_World * T_Camera_Marker
    # And T_Camera_Marker is the inverse of T_Marker_Camera

    # Calculate inverse of T_Marker_Camera
    # Inverse rotation: R_inv = R.T
    # Inverse translation: t_inv = -R.T @ t
    # Or simply use numpy's matrix inverse:
    T_marker_camera_inv_mat = np.linalg.inv(T_marker_camera_mat)
    # T_Camera_Marker_mat = T_marker_camera_inv_mat # Alias for clarity

    # Calculate T_Camera_World = T_Marker_World * T_Marker_Camera_inverse
    T_camera_world_mat = T_marker_world_mat @ T_marker_camera_inv_mat

    # --- Extract Result ---
    t_camera_world = T_camera_world_mat[:3, 3]
    R_camera_world_mat = T_camera_world_mat[:3, :3]
    # Convert rotation matrix back to quaternion [x, y, z, w]
    q_camera_world = Rotation.from_matrix(R_camera_world_mat).as_quat()

    # --- Print Results ---
    print("\n--- Calculated Camera Pose (T_Camera_World) ---")
    print(f" Frame '{args.camera_frame}' relative to Frame '{args.world_frame}'")
    print(f" Translation [x y z]:    {t_camera_world[0]:.4f} {t_camera_world[1]:.4f} {t_camera_world[2]:.4f}")
    print(f" Quaternion [x y z w]: {q_camera_world[0]:.4f} {q_camera_world[1]:.4f} {q_camera_world[2]:.4f} {q_camera_world[3]:.4f}")
    print("-" * 50)

    # --- Output Static Transform Publisher Command ---
    print("\nExample command for static_transform_publisher (adjust frames if needed):")
    t = t_camera_world
    q = q_camera_world # Assuming this is [qx, qy, qz, qw]
    print(f"ros2 run tf2_ros static_transform_publisher \\")
    print(f"  --x {t[0]:.4f} --y {t[1]:.4f} --z {t[2]:.4f} \\")
    print(f"  --qx {q[0]:.4f} --qy {q[1]:.4f} --qz {q[2]:.4f} --qw {q[3]:.4f} \\")
    print(f"  --frame-id world --child-frame-id camera_link")
    print("--------------------------------------------------")


if __name__ == "__main__":
    main()
