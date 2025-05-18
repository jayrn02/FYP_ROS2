#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from moveit_py.moveit_py import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_py.planning import PlanRequestParameters

# Import tf2 utilities if needed for frame transforms (not strictly needed here)
# from tf2_ros import Buffer, TransformListener
# from tf2_geometry_msgs import do_transform_pose_stamped

import time

class ObjectFollower(Node):
    def __init__(self):
        super().__init__("object_follower_node")
        self.logger = get_logger("object_follower")

        # --- MoveItPy Setup ---
        # Using MoveItConfigsBuilder to load parameters is generally recommended
        # Adjust package_name and config paths as needed for your setup
        moveit_configs = (
            MoveItConfigsBuilder("robot", package_name="moveitv2")
            .robot_description(file_path="config/robot.urdf") # Make sure paths are correct
            .robot_description_semantic(file_path="config/robot.srdf")
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .planning_pipelines(default_planning_pipeline="ompl") # Specify your default pipeline
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .joint_limits(file_path="config/joint_limits.yaml")
            .to_moveit_configs()
        ).to_dict()

        # Initialize MoveItPy instance providing parameters loaded above
        # Use a separate node name for the MoveItPy internal node
        self.moveit = MoveItPy(node_name="moveit_py_planning", config_dict=moveit_configs)
        self.logger.info("MoveItPy instance created.")

        # Get Planning Component for the arm group
        self.arm_planning_component = self.moveit.get_planning_component("arm")
        if not self.arm_planning_component:
             self.logger.error("Could not get planning component for 'arm'. Check group name in SRDF.")
             rclpy.shutdown()
             return
        self.logger.info("Got planning component for group 'arm'.")

        # Define the planning parameters - these are optional but recommended
        self.plan_params = PlanRequestParameters(self, "arm") # Use node and group name
        # Example: self.plan_params.set_parameter("planning_attempts", 10)
        # Example: self.plan_params.set_parameter("planning_time", 5.0)

        # Get the link name which the pose goal refers to (tip of the planning group)
        self.end_effector_link = self.arm_planning_component.get_end_effector_link_name()
        if not self.end_effector_link:
             self.logger.error("Could not get end effector link name for group 'arm'. Check SRDF.")
             # Defaulting based on previous discussion, but this should be reliable
             self.end_effector_link = "end_effector"
             self.logger.warn(f"Defaulting end_effector_link to '{self.end_effector_link}'.")
        else:
            self.logger.info(f"Planning for end effector link: '{self.end_effector_link}'")


        # --- TF2 Setup (Optional but good practice) ---
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Subscriber ---
        self.subscription = self.create_subscription(
            PointStamped,
            '/detected_object/position_world',
            self.position_callback,
            10  # QoS profile depth
        )
        self.logger.info("Subscribed to /detected_object/position_world")

        # Flag to prevent concurrent planning if callback is rapid
        self.is_planning = False


    def position_callback(self, msg: PointStamped):
        if self.is_planning:
            self.logger.warn("Already planning, skipping new goal.")
            return

        self.is_planning = True
        self.logger.info(f"Received point in frame '{msg.header.frame_id}': "
                         f"X={msg.point.x:.3f}, Y={msg.point.y:.3f}, Z={msg.point.z:.3f}")

        # --- Goal Pose Construction ---
        target_pose = PoseStamped()
        target_pose.header.frame_id = msg.header.frame_id # Should be 'base_link'
        target_pose.header.stamp = self.get_clock().now().to_msg()

        # Position from the message
        target_pose.pose.position.x = msg.point.x
        target_pose.pose.position.y = msg.point.y
        target_pose.pose.position.z = msg.point.z

        # Orientation: Keep it fixed to maintain parallel constraint
        # Assuming identity quaternion (0,0,0,1) in base_link keeps it parallel.
        # Adjust this if your 'parallel' orientation requires a different quaternion.
        target_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # --- TF Transformation (If needed) ---
        # planning_frame = self.moveit.get_planning_frame() # Usually 'base_link' or 'world'
        # if target_pose.header.frame_id != planning_frame:
        #     try:
        #         # Wait for transform an dperform it
        #         # transform = self.tf_buffer.lookup_transform(planning_frame, target_pose.header.frame_id, rclpy.time.Time())
        #         # target_pose = do_transform_pose_stamped(target_pose, transform)
        #         self.logger.info(f"Transformed goal pose to planning frame '{planning_frame}'")
        #     except Exception as e:
        #         self.logger.error(f"Could not transform goal pose: {e}")
        #         self.is_planning = False
        #         return

        # --- Plan and Execute ---
        try:
            self.logger.info(f"Setting goal pose for link '{self.end_effector_link}': {target_pose.pose.position}")

            # Set start state before planning
            self.arm_planning_component.set_start_state_to_current_state()

            # Set the goal state
            self.arm_planning_component.set_goal_state(
                pose_stamped_msg=target_pose,
                pose_link=self.end_effector_link
            )

            # Plan to goal
            self.logger.info("Planning trajectory...")
            # plan_solution = self.arm_planning_component.plan() # Without parameters
            plan_solution = self.arm_planning_component.plan(self.plan_params) # With parameters

            if plan_solution:
                self.logger.info("Planning successful. Executing trajectory...")
                # Execute the planned trajectory
                # MoveItPy's execute usually figures out the controller automatically
                self.moveit.execute(plan_solution, controllers=[]) # Empty list often works
                self.logger.info("Execution complete.")
            else:
                self.logger.error("Planning failed!")

        except Exception as e:
            self.logger.error(f"Error during planning or execution: {e}")
        finally:
            # Allow next goal processing
            self.is_planning = False


def main(args=None):
    rclpy.init(args=args)
    node = ObjectFollower()
    # Give MoveItPy a moment to fully initialize, especially the planning scene monitor
    time.sleep(2.0)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()