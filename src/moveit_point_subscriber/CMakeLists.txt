cmake_minimum_required(VERSION 3.8)
project(moveit_point_subscriber)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED) # For error codes, display trajectory etc.

# Add the executable
add_executable(point_goal_subscriber_node src/point_goal_subscriber_node.cpp)

# Link the executable against dependencies
ament_target_dependencies(point_goal_subscriber_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs

  moveit_msgs
)

# Install the executable
install(TARGETS
  point_goal_subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)

# Add linting and testing support (optional but recommended)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in Galactic+)
  # uncomment the line when this package is not cpplint compliant
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize package processing
ament_package()