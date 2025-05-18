cmake_minimum_required(VERSION 3.8)
project(uno_stepper_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
# serial_driver removed
find_package(Boost REQUIRED COMPONENTS system)

# Add include directories
include_directories(include ${Boost_INCLUDE_DIRS}) # Keep Boost includes global

# Build the shared library
add_library(uno_stepper_interface SHARED
  src/uno_stepper_interface.cpp
)

# Handle ROS dependencies (includes & links) using the ament helper
ament_target_dependencies(uno_stepper_interface
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  # serial_driver removed
)

# <<< MODIFIED: Link Boost using PLAIN signature (no PUBLIC) >>>
target_link_libraries(uno_stepper_interface
    ${Boost_LIBRARIES} # Link against the libraries found by find_package(Boost...)
)

# <<< REMOVED explicit target_include_directories for Boost >>>
# Let's rely on the global include_directories() for Boost includes for now.
# If headers aren't found later, we can add this back.
# target_include_directories(uno_stepper_interface PUBLIC
#   ${Boost_INCLUDE_DIRS}
# )

# Install the library
install(TARGETS uno_stepper_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header file
install(
    DIRECTORY include/
    DESTINATION include
)

# Install the plugin description file
install(FILES uno_stepper_interface_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export plugin for hardware_interface
pluginlib_export_plugin_description_file(hardware_interface uno_stepper_interface_plugin.xml)

# --- Testing removed ---

# --- Final ---
ament_package()