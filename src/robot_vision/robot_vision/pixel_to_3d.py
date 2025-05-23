#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration

import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PointStamped, Point, TransformStamped
from sensor_msgs.msg import CameraInfo
import image_geometry # From vision_opencv/image_geometry

import numpy as np

class PixelTo3DNode(Node):
    def __init__(self):
        super().__init__('pixel_to_3d_node')
        self.get_logger().info('Pixel to 3D Node Started')

        # --- Parameters ---
        self.declare_parameter('target_frame', 'base_link') # Frame to transform the 3D point into
        self.target_frame = self.get_parameter('target_frame').value

        # --- QoS Profiles ---
        # Reliable profile for CameraInfo - expect it once or infrequently
        camera_info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL # Makes sure we get the last published message
        )

        # --- Subscribers ---
        self.pixel_sub = self.create_subscription(
            PointStamped,
            '/detected_object/center_pixels', # Input: Pixel coordinates from detector
            self.pixel_callback,
            10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',          # Input: Camera calibration
            self.camera_info_callback,
            10)                 # Use reliable QoS for camera info

        # --- Publisher ---
        self.world_point_pub = self.create_publisher(
            PointStamped,
            '/detected_object/position_world', # Output: 3D point in target_frame
            10)

        # --- TF2 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- State ---
        self.camera_model = None # Stores the image_geometry camera model

    def camera_info_callback(self, msg):
        if self.camera_model is None:
            self.get_logger().info(f"Received CameraInfo for '{msg.header.frame_id}'")
            self.camera_model = image_geometry.PinholeCameraModel()
            self.camera_model.from_camera_info(msg) # Changed from fromCameraInfo
        # Optional: Could unsubscribe after receiving info if it's truly static
        # self.destroy_subscription(self.camera_info_sub)

    def pixel_callback(self, msg: PointStamped):
        if self.camera_model is None:
            self.get_logger().warn('Waiting for CameraInfo...')
            return

        # --- Get Pixel Coordinates ---
        # Assuming detector publishes pixels in PointStamped x, y fields
        pixel_x = msg.point.x
        pixel_y = msg.point.y
        self.get_logger().debug(f"Received pixel: ({pixel_x}, {pixel_y}) in frame {msg.header.frame_id}")

        # --- Calculate 3D Point in Camera Frame ---
        # Project pixel into a 3D ray (direction vector) in the camera frame
        # Z=1 is arbitrary, we just need the direction
        ray_camera = self.camera_model.project_pixel_to_3d_ray((pixel_x, pixel_y)) # Changed from projectPixelTo3dRay
        # ray_camera is a unit vector (x, y, z) pointing from camera center through pixel

        # --- Get Transform from Camera Frame to Target Frame ---
        camera_frame = msg.header.frame_id # Should be 'default_cam'
        try:
            # Look up the transform (wait up to 1 sec)
            # Need transform FROM camera TO target frame
            transform_stamped = self.tf_buffer.lookup_transform(
                self.target_frame,      # Target frame (e.g., 'base_link')
                camera_frame,           # Source frame (e.g., 'default_cam')
                rclpy.time.Time(),      # Get latest available transform
                timeout=Duration(seconds=1.0)
            )
            self.get_logger().debug(f"Transform {camera_frame} -> {self.target_frame} found.")

        except TransformException as ex:
            self.get_logger().error(f'Could not transform {camera_frame} to {self.target_frame}: {ex}')
            return

        # --- Assume Workspace Plane Z=0 in Target Frame (base_link) ---
        # We need to find where the ray intersects the Z=0 plane in the target_frame
        # The ray starts at the camera's origin (transformed to target_frame)
        # and goes in the direction of ray_camera (transformed to target_frame).

        # 1. Camera origin in target_frame
        cam_origin_target = Point(
            x=transform_stamped.transform.translation.x,
            y=transform_stamped.transform.translation.y,
            z=transform_stamped.transform.translation.z
        )

        # 2. Ray direction vector in target_frame
        # We need to rotate ray_camera using the transform's rotation
        # Use numpy and scipy for quaternion rotation (ensure installed: pip install numpy scipy)
        from scipy.spatial.transform import Rotation as R
        quat_tf = transform_stamped.transform.rotation
        rotation = R.from_quat([quat_tf.x, quat_tf.y, quat_tf.z, quat_tf.w])
        ray_target = rotation.apply(ray_camera) # Rotates the direction vector

        # 3. Calculate intersection with Z=0 plane in target_frame
        # Parametric equation of the line: P = Origin + t * Direction
        # P = cam_origin_target + t * ray_target
        # We want the point P where P.z = 0
        # cam_origin_target.z + t * ray_target.z = 0
        # t = -cam_origin_target.z / ray_target.z

        if abs(ray_target[2]) < 1e-6: # Avoid division by zero if ray is parallel to Z=0 plane
             self.get_logger().warn("Camera ray is parallel to the target Z=0 plane, cannot calculate intersection.")
             return

        t = -cam_origin_target.z / ray_target[2]

        if t < 0: # Intersection point is behind the camera
             self.get_logger().warn("Intersection point is behind the camera.")
             return

        # Calculate the intersection point P
        intersection_point = Point()
        intersection_point.x = cam_origin_target.x + t * ray_target[0]
        intersection_point.y = cam_origin_target.y + t * ray_target[1]
        intersection_point.z = cam_origin_target.z + t * ray_target[2] # Should be very close to 0

        # --- Publish the 3D Point ---
        point_world_msg = PointStamped()
        point_world_msg.header.stamp = msg.header.stamp # Use same timestamp as pixel
        point_world_msg.header.frame_id = self.target_frame # Frame is base_link
        point_world_msg.point = intersection_point

        self.world_point_pub.publish(point_world_msg)
        self.get_logger().debug(f"Published world point: ({intersection_point.x:.3f}, {intersection_point.y:.3f}, {intersection_point.z:.3f}) in {self.target_frame}")


def main(args=None):
    rclpy.init(args=args)
    pixel_to_3d_node = PixelTo3DNode()
    rclpy.spin(pixel_to_3d_node)
    # Destroy the node explicitly
    pixel_to_3d_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
