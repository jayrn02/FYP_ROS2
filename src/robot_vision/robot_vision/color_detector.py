#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Point # Make sure Point is imported
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__('color_detector_node')
        self.get_logger().info('Color Detector Node Started')

        # --- Parameters ---
        # TODO: Define HSV color range parameters (make them adjustable later)
        # Example for a bright red - YOU WILL NEED TO TUNE THESE
        self.declare_parameter('hsv_lower_hue1', 0)
        self.declare_parameter('hsv_lower_sat', 120)
        self.declare_parameter('hsv_lower_val', 70)
        self.declare_parameter('hsv_upper_hue1', 10)
        self.declare_parameter('hsv_upper_sat', 255)
        self.declare_parameter('hsv_upper_val', 255)
        # Red wraps around 180 in OpenCV HSV, so potentially a second range
        self.declare_parameter('hsv_lower_hue2', 170)
        self.declare_parameter('hsv_upper_hue2', 180)
        # Minimum contour area to filter noise
        self.declare_parameter('min_contour_area', 500.0)


        # --- Subscribers ---
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_rect', # Subscribe to the rectified image
            self.image_callback,
            10) # QoS profile depth

        # --- Publishers ---
        self.center_pub = self.create_publisher(
            PointStamped,
            '/detected_object/center_pixels',
            10)
        self.debug_image_pub = self.create_publisher(
            Image,
            '/object_detector/debug_image',
            10)

        # --- CV Bridge ---
        self.bridge = CvBridge()

    def image_callback(self, msg):
        self.get_logger().debug('Received image')
        try:
            # Convert ROS Image message to OpenCV image (BGR format)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge Error: {e}')
            return

        # --- Get Parameters ---
        hsv_lower_hue1 = self.get_parameter('hsv_lower_hue1').value
        hsv_lower_sat = self.get_parameter('hsv_lower_sat').value
        hsv_lower_val = self.get_parameter('hsv_lower_val').value
        hsv_upper_hue1 = self.get_parameter('hsv_upper_hue1').value
        hsv_upper_sat = self.get_parameter('hsv_upper_sat').value
        hsv_upper_val = self.get_parameter('hsv_upper_val').value
        hsv_lower_hue2 = self.get_parameter('hsv_lower_hue2').value
        hsv_upper_hue2 = self.get_parameter('hsv_upper_hue2').value
        min_area = self.get_parameter('min_contour_area').value


        # ==========================================================
        # === START OPENCV PROCESSING HERE ===
        # ==========================================================

        # 1. Convert BGR to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 2. Define HSV color range(s)
        #    (Example for red, requires tuning!)
        lower_bound1 = np.array([hsv_lower_hue1, hsv_lower_sat, hsv_lower_val])
        upper_bound1 = np.array([hsv_upper_hue1, hsv_upper_sat, hsv_upper_val])
        # Second range for red wrap-around
        lower_bound2 = np.array([hsv_lower_hue2, hsv_lower_sat, hsv_lower_val])
        upper_bound2 = np.array([hsv_upper_hue2, hsv_upper_sat, hsv_upper_val])


        # 3. Create mask(s)
        mask1 = cv2.inRange(hsv_image, lower_bound1, upper_bound1)
        # If detecting red, combine masks (otherwise just use mask1)
        mask2 = cv2.inRange(hsv_image, lower_bound2, upper_bound2)
        mask = cv2.bitwise_or(mask1, mask2) # Comment this out if not detecting red

        # --- Optional: Noise Reduction ---
        # kernel = np.ones((5,5),np.uint8)
        # mask = cv2.erode(mask, kernel, iterations = 1)
        # mask = cv2.dilate(mask, kernel, iterations = 1)


        # 4. Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 5. Filter contours and find the largest one
        largest_contour = None
        max_area = 0
        detected = False

        if contours:
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > min_area and area > max_area:
                    max_area = area
                    largest_contour = contour
                    detected = True

        # --- Prepare Debug Image ---
        # Draw on a copy of the original image
        debug_image = cv_image.copy()


        # 6. Calculate center and publish if detected
        if detected and largest_contour is not None:
            # Calculate moments for the largest contour
            M = cv2.moments(largest_contour)

            # Calculate centroid (handle division by zero)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # Draw contour and center on debug image
                cv2.drawContours(debug_image, [largest_contour], -1, (0, 255, 0), 2) # Green contour
                cv2.circle(debug_image, (cx, cy), 7, (0, 0, 255), -1) # Red filled circle center
                cv2.putText(debug_image, f"({cx}, {cy})", (cx - 40, cy - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                # Publish the center point
                point_msg = PointStamped()
                point_msg.header.stamp = msg.header.stamp # Use image timestamp
                point_msg.header.frame_id = msg.header.frame_id # Should be 'default_cam'
                point_msg.point = Point(x=float(cx), y=float(cy), z=0.0) # Store pixels in x,y
                self.center_pub.publish(point_msg)
                self.get_logger().debug(f'Object detected at pixel: ({cx}, {cy})')

            else:
                self.get_logger().warn('Detected contour M[m00] is zero, cannot calculate center.')
        else:
             self.get_logger().debug('No object detected')


        # 7. Publish debug image
        try:
            debug_image_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
            debug_image_msg.header = msg.header # Keep same timestamp and frame_id
            self.debug_image_pub.publish(debug_image_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge Error publishing debug image: {e}')

        # ==========================================================
        # === END OPENCV PROCESSING ===
        # ==========================================================


def main(args=None):
    rclpy.init(args=args)
    color_detector_node = ColorDetectorNode()
    rclpy.spin(color_detector_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    color_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
