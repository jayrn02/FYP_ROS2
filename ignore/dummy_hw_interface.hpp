#ifndef DUMMY_HW_INTERFACE__DUMMY_HW_INTERFACE_HPP_
#define DUMMY_HW_INTERFACE__DUMMY_HW_INTERFACE_HPP_

#include <vector>
#include <string>
#include <map> // For joint name mapping
#include <cmath> // For M_PI

// ROS 2 and ros2_control includes
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp" // Include for logging

// Serial Driver include
#include "serial_driver/serial_driver.hpp" // Include the serial driver header

namespace dummy_hw_interface
{

class DummyHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DummyHardwareInterface)

  // Standard ros2_control hardware interface methods
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Store hardware commands and states
  std::vector<double> hw_commands_positions_; // Renamed for clarity
  std::vector<double> hw_states_positions_;   // Renamed for clarity
  std::vector<double> hw_states_velocities_;  // Renamed for clarity

  // Serial Port Communication
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  std::string serial_port_name_;
  uint32_t serial_baud_rate_;
  // Add other serial parameters if needed (flow control, parity, etc.)

  // Configuration parameters from hardware info
  // (Example: could store serial port name, baud rate here if passed via xacro)

  // Conversion factors and constants (based on your provided info)
  // Arm Joints (Base, Shoulder, Elbow)
  const double ARM_STEPS_PER_RADIAN = 2291.83;
  const int BASE_HOME_STEPS = 3640;
  const int SHOULDER_HOME_STEPS = 1780;
  const int ELBOW_HOME_STEPS = 665;

  // Gripper Joint
  const double GRIPPER_STEPS_PER_METER = 70588.2;
  const int GRIPPER_MAX_STEPS = 1200; // Max steps from Arduino config
  const double GRIPPER_MAX_METERS = 0.017; // Max meters from URDF

  // Map ROS joint names to Arduino identifiers/indices if needed
  // (Could also use the order from info_.joints)
  std::map<std::string, int> joint_name_to_arduino_index_; // Example mapping

  // Helper function for serial communication (optional but recommended)
  bool sendSerialCommand(const std::string& command);
  std::string readSerialResponse(); // Needs implementation details

  // Helper functions for unit conversions
  int radiansToSteps(const std::string& joint_name, double radians);
  double stepsToRadians(const std::string& joint_name, int steps);
  int metersToSteps(double meters);
  double stepsToMeters(int steps);

};

} // namespace dummy_hw_interface

#endif // DUMMY_HW_INTERFACE__DUMMY_HW_INTERFACE_HPP_