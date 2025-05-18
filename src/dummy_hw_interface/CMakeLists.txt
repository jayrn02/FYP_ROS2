cmake_minimum_required(VERSION 3.8)
project(dummy_hw_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(serial_driver REQUIRED)

# Add the source file to a library target
add_library(${PROJECT_NAME} SHARED
  src/dummy_hw_interface.cpp
)

# Link against dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  serial_driver
)

# Add this block to specify include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  # Explicitly add include directories from serial_driver
  $<BUILD_INTERFACE:${serial_driver_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:${serial_driver_INCLUDE_DIRS}>
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the header file
install(DIRECTORY include/
  DESTINATION include/
)

# Export and install the plugin description file
pluginlib_export_plugin_description_file(hardware_interface dummy_hw_interface_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Standard ROS 2 package setup
ament_package()
