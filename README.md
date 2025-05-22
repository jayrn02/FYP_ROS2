# ROS2 Robot Arm Control and Vision Project

This repository contains the software for controlling a custom-built 3-DOF (Degrees of Freedom) robotic arm with a gripper, integrated with ROS2 for high-level control, MoveIt for motion planning, and a computer vision package for perception tasks.

## Overview

The system is composed of several key components:

*   **Low-Level Control (Arduino):**
    *   An Arduino board (running the code in `ROS2_Controlv3/ROS2_Controlv3.ino`) is responsible for the direct control of the stepper motors for the robot's base, shoulder, and elbow joints, as well as the BYJ stepper-based gripper.
    *   It communicates with the ROS2 system via serial communication, receiving motion commands and sending status updates.
    *   Includes a homing sequence for robot initialization.

*   **ROS2 Ecosystem:**
    *   Serves as the central framework for communication and process management between different parts of the system.
    *   The `src/` directory contains various ROS2 packages.

*   **Motion Planning (MoveIt):**
    *   The `src/moveitv3` package contains the configuration and launch files for using the MoveIt Motion Planning Framework.
    *   This allows for complex motion planning, collision avoidance (if configured), and kinematic calculations.
    *   It likely uses the robot description found in `src/three_dof_robot_description`.

*   **Robot Vision:**
    *   The `src/robot_vision` package provides capabilities for processing input from a camera.
    *   This can be used for tasks such as object detection, color detection, or providing 3D coordinates for objects to the motion planning system.
    *   It utilizes libraries like OpenCV and `image_pipeline`.

*   **Robot Description:**
    *   The `src/three_dof_robot_description` package (as inferred from its name and usage in MoveIt) likely contains the URDF (Unified Robot Description Format) files that define the robot's physical structure, joints, and links.

## Functionality

The typical workflow involves:
1.  The Arduino firmware initializes and homes the robot arm.
2.  ROS2 nodes launch, including MoveIt for motion planning and the robot vision system.
3.  The vision system might identify a target or a point of interest.
4.  This information can be passed to MoveIt, which plans a trajectory for the robot arm.
5.  MoveIt sends joint commands through `ros2_control` (or a similar interface) which are then translated into low-level commands for the Arduino.
6.  The Arduino executes these commands to move the stepper motors.

This repository is a comprehensive project for a robotic arm, suitable for learning, research, or hobbyist robotics development.
