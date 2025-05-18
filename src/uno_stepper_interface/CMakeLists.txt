cmake_minimum_required(VERSION 3.8)
project(uno_stepper_interface)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED) # Ensure pluginlib is found
find_package(Boost REQUIRED system)

# Add the library
add_library(${PROJECT_NAME} SHARED
  src/uno_stepper_interface.cpp
)

# Target dependencies
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib # Add pluginlib dependency
)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS} # Add Boost include directory
)

# Link libraries
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${Boost_LIBRARIES} # Link Boost system library
)

# --- Pluginlib Export ---
# Reference the CORRECT XML filename here, just like in dummy_hw_interface
pluginlib_export_plugin_description_file(hardware_interface uno_stepper_interface_plugin.xml)

# --- Installation ---
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# The install(FILES uno_stepper_interface_plugin.xml ...) command might be redundant
# when using pluginlib_export_plugin_description_file, you can try removing it
# if you added it previously based on earlier suggestions. If unsure, leave it for now.
# install(FILES uno_stepper_interface_plugin.xml
#  DESTINATION share/${PROJECT_NAME}
# )

# Install headers needed by downstream packages (if any)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface pluginlib) # Add pluginlib here too

ament_package()
