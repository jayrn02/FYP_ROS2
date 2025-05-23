#ifndef ARDUINO_HW_INTERFACE__ARDUINO_HW_INTERFACE_HPP_
#define ARDUINO_HW_INTERFACE__ARDUINO_HW_INTERFACE_HPP_

#include <vector>
#include <string>
#include <map>
#include <chrono>
#include <cmath> // For std::isnan, M_PI

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "serial_driver/serial_driver.hpp"
#include "io_context/io_context.hpp"

namespace arduino_hw_interface
{

class ArduinoHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArduinoHardwareInterface)

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
  // Storage for commands and states
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  std::vector<double> prev_hw_states_positions_; // For velocity calculation

  // Serial Port Communication
  drivers::common::IoContext io_context_{1}; 
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  std::string serial_port_name_;
  uint32_t serial_baud_rate_;
  
  // Configurable timeouts and delays
  std::chrono::milliseconds serial_read_timeout_{500};
  std::chrono::milliseconds arduino_ready_timeout_{10000};
  std::chrono::milliseconds post_open_delay_{1000};


  // Conversion factors and constants (corrected based on analysis)
  // These should ideally match Arduino's config.h after verification
  const double ARM_STEPS_PER_RADIAN = 2291.83; // From existing dummy_hw_interface.hpp
  const int BASE_HOME_STEPS_OFFSET = 3740;    // Arduino Z_HOME_STEPS
  const int SHOULDER_HOME_STEPS_OFFSET = 1720;// Arduino Y_HOME_STEPS
  const int ELBOW_HOME_STEPS_OFFSET = 680;   // Arduino X_HOME_STEPS

  const double GRIPPER_STEPS_PER_METER = 70588.2; // From existing dummy_hw_interface.hpp
  const int GRIPPER_MAX_STEPS = 1200;             // Arduino BYJ_GRIP_STEPS
  // const double GRIPPER_MAX_METERS = 0.017;     // URDF limit, for reference

  // Joint mapping and ordering
  std::map<std::string, size_t> joint_name_to_index_;
  std::vector<std::string> joint_names_in_arduino_order_; // Stores the 4 controlled joint names in Arduino's expected order

  // Helper functions
  bool sendSerialCommand(const std::string& command);
  std::string receiveSerialLine(std::chrono::milliseconds timeout); 

  int rosToArduinoSteps(const std::string& joint_name, double ros_value);
  double arduinoToRosUnits(const std::string& joint_name, int arduino_steps);
};

} // namespace arduino_hw_interface

#endif // ARDUINO_HW_INTERFACE__ARDUINO_HW_INTERFACE_HPP_
