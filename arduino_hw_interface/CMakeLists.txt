cmake_minimum_required(VERSION 3.8)
project(arduino_hw_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(io_context REQUIRED)
find_package(serial_driver REQUIRED)

add_library(arduino_hw_interface SHARED
  src/arduino_hw_interface.cpp
)

ament_target_dependencies(arduino_hw_interface
  rclcpp
  hardware_interface
  pluginlib
  io_context
  serial_driver
)

install(TARGETS arduino_hw_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include
)

pluginlib_export_plugin_description_file(hardware_interface src/arduino_hw_interface.xml)

ament_package()
