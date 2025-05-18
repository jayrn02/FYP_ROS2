// --- START OF FILE dummy_hw_interface.cpp ---

#include "dummy_hw_interface/dummy_hw_interface.hpp"

#include <vector>
#include <string>
#include <limits>
#include <chrono> // For timeouts/sleeps
#include <thread> // For sleeps
#include <stdexcept> // For exceptions
#include <sstream> // For string manipulation
#include <cmath>   // For std::isnan (optional but harmless)
#include <memory>  // For std::make_unique/make_shared
#include <algorithm> // For std::max/min

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp" // Make sure this is included
#include "io_context/io_context.hpp"       // Header included via dummy_hw_interface.hpp usually

// Use drivers namespace for convenience
using namespace drivers::serial_driver;
using drivers::common::IoContext; // Explicitly use IoContext from common namespace

namespace dummy_hw_interface
{

// --- Start Helper Functions ---

// Converts radians (ROS) to steps (Arduino) for arm joints
int DummyHardwareInterface::radiansToSteps(const std::string& joint_name, double radians)
{
    int steps = 0;
    // --- Constants defined in header ---
    if (joint_name == "base_link_to_upper_base_link") {
        steps = static_cast<int>(radians * ARM_STEPS_PER_RADIAN) + BASE_HOME_STEPS;
    } else if (joint_name == "upper_base_link_to_lower_link") {
        steps = static_cast<int>(radians * ARM_STEPS_PER_RADIAN) + SHOULDER_HOME_STEPS;
    } else if (joint_name == "lower_link_to_upper_link") {
        steps = static_cast<int>(radians * ARM_STEPS_PER_RADIAN) + ELBOW_HOME_STEPS;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Unknown arm joint name for radiansToSteps: %s", joint_name.c_str());
    }
    return steps;
}

// Converts steps (Arduino) to radians (ROS) for arm joints
double DummyHardwareInterface::stepsToRadians(const std::string& joint_name, int steps)
{
    double radians = 0.0;
     // --- Constants defined in header ---
     if (joint_name == "base_link_to_upper_base_link") {
        radians = static_cast<double>(steps - BASE_HOME_STEPS) / ARM_STEPS_PER_RADIAN;
    } else if (joint_name == "upper_base_link_to_lower_link") {
        radians = static_cast<double>(steps - SHOULDER_HOME_STEPS) / ARM_STEPS_PER_RADIAN;
    } else if (joint_name == "lower_link_to_upper_link") {
        radians = static_cast<double>(steps - ELBOW_HOME_STEPS) / ARM_STEPS_PER_RADIAN;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Unknown arm joint name for stepsToRadians: %s", joint_name.c_str());
    }
    return radians;
}

// Converts meters (ROS) to steps (Arduino) for gripper joint
int DummyHardwareInterface::metersToSteps(double meters)
{
    // --- Constants defined in header ---
    meters = std::max(0.0, std::min(meters, GRIPPER_MAX_METERS));
    int steps = static_cast<int>(meters * GRIPPER_STEPS_PER_METER);
    steps = std::max(0, std::min(steps, GRIPPER_MAX_STEPS));
    return steps;
}

// Converts steps (Arduino) to meters (ROS) for gripper joint
double DummyHardwareInterface::stepsToMeters(int steps)
{
    // --- Constants defined in header ---
    steps = std::max(0, std::min(steps, GRIPPER_MAX_STEPS));
    double meters = static_cast<double>(steps) / GRIPPER_STEPS_PER_METER;
    meters = std::max(0.0, std::min(meters, GRIPPER_MAX_METERS));
    return meters;
}


bool DummyHardwareInterface::sendSerialCommand(const std::string& command)
{
    if (!serial_driver_ || !serial_driver_->port()->is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Serial port not open or driver not initialized, cannot send command.");
        return false;
    }
    try {
        RCLCPP_DEBUG(rclcpp::get_logger("DummyHardwareInterface"), "Sending command: %s", command.c_str());
        std::vector<uint8_t> data(command.begin(), command.end());
        serial_driver_->port()->send(data);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Error sending serial command: %s", e.what());
        return false;
    }
}

std::string DummyHardwareInterface::readSerialResponse()
{
     if (!serial_driver_ || !serial_driver_->port()->is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"),
                     "Serial port not open or driver not initialized, cannot read response.");
        return "";
    }
    std::string acc;
    std::vector<uint8_t> chunk(1); // Read one byte at a time
    auto start = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::milliseconds(500); // Read timeout

    try {
        while (true) {
            // Check for timeout
            if (std::chrono::steady_clock::now() - start > timeout) {
                if (!acc.empty()) { // Log accumulated data if timeout occurs mid-message
                     RCLCPP_WARN(
                        rclcpp::get_logger("DummyHardwareInterface"),
                        "Timeout waiting for newline in readSerialResponse. Accumulated: '%s'",
                        acc.c_str()
                     );
                } else {
                     RCLCPP_WARN(
                        rclcpp::get_logger("DummyHardwareInterface"),
                        "Timeout waiting for any data in readSerialResponse."
                     );
                }
                return ""; // Return empty on timeout
            }

            // Attempt to receive a byte
            size_t n = serial_driver_->port()->receive(chunk);
            if (n > 0) {
                char c = static_cast<char>(chunk[0]);
                acc += c;
                // Check if we received the delimiter (newline)
                if (c == '\n') {
                    RCLCPP_DEBUG(rclcpp::get_logger("DummyHardwareInterface"), "Read line: %s", acc.c_str());
                    return acc; // Return the complete line
                }
                // Reset timeout start time after receiving data to ensure timeout applies to gaps
                start = std::chrono::steady_clock::now();
            } else {
                // No data received, yield/sleep briefly to avoid busy-waiting
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    } catch (const std::exception & e) {
        RCLCPP_ERROR(
          rclcpp::get_logger("DummyHardwareInterface"),
          "Exception during serial read: %s. Accumulated: '%s'",
          e.what(), acc.c_str()
        );
        return ""; // Return empty on exception
    }
}

// --- END Helper Functions ---


// INITIALIZATION //

hardware_interface::CallbackReturn DummyHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "on_init starting");

  // --- Read parameters from hardware info (URDF/XACRO) ---
  try {
      serial_port_name_ = info_.hardware_parameters.at("serial_port");
      serial_baud_rate_ = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("baud_rate")));
      RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Serial Port: %s, Baud Rate: %u", serial_port_name_.c_str(), serial_baud_rate_);
  } catch (const std::out_of_range & ex) {
      RCLCPP_FATAL(rclcpp::get_logger("DummyHardwareInterface"), "Parameters 'serial_port' or 'baud_rate' not found in URDF hardware parameters. Check your <hardware> tag.");
      return hardware_interface::CallbackReturn::ERROR;
  } catch (const std::invalid_argument & ex) {
      RCLCPP_FATAL(rclcpp::get_logger("DummyHardwareInterface"), "Parameter 'baud_rate' is not a valid unsigned integer.");
      return hardware_interface::CallbackReturn::ERROR;
  } catch (const std::exception & ex) {
       RCLCPP_FATAL(rclcpp::get_logger("DummyHardwareInterface"), "Error reading hardware parameters: %s", ex.what());
       return hardware_interface::CallbackReturn::ERROR;
  }

  // --- Initialize Serial Driver ---
  try {
      // IoContext io_context_ is already constructed as a member variable.

      // Configure serial port settings (8N1, no flow control)
      // Note: std::make_shared is fine for the config object itself
      auto config = std::make_shared<drivers::serial_driver::SerialPortConfig>(serial_baud_rate_,
                                                                               drivers::serial_driver::FlowControl::NONE,
                                                                               drivers::serial_driver::Parity::NONE,
                                                                               drivers::serial_driver::StopBits::ONE);

      // Create SerialDriver, passing a REFERENCE to the member io_context_
      serial_driver_ = std::make_unique<drivers::serial_driver::SerialDriver>(io_context_); // Pass by reference

      // Initialize the port with the device name and config
      serial_driver_->init_port(serial_port_name_, *config); // Pass config object by value/copy

      // Note: Port is not opened here, but in on_activate
      RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Serial driver initialized for port %s.", serial_port_name_.c_str());
  } catch (const std::exception& e) {
       RCLCPP_FATAL(rclcpp::get_logger("DummyHardwareInterface"), "Failed during serial driver setup: %s", e.what());
       serial_driver_.reset(); // Clean up driver object if creation failed partially
       // No need to reset io_context_, it's a member and will be destroyed automatically
       return hardware_interface::CallbackReturn::ERROR;
  }

  // --- Resize and initialize storage vectors ---
  size_t num_joints = info_.joints.size();
  hw_commands_positions_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
  hw_states_positions_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(num_joints, std::numeric_limits<double>::quiet_NaN()); // Keep velocity


  // --- Validate Joint Interfaces ---
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Command interface check (expecting only position)
    if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(rclcpp::get_logger("DummyHardwareInterface"), "Joint '%s' must have exactly one command interface named '%s'. Check your URDF.",
                   joint.name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // State interface check (expecting position AND velocity as per Option 1)
    if (joint.state_interfaces.size() != 2) {
         RCLCPP_FATAL(rclcpp::get_logger("DummyHardwareInterface"), "Joint '%s' must have exactly two state interfaces ('%s' and '%s'). Check your URDF.",
                      joint.name.c_str(), hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY);
         return hardware_interface::CallbackReturn::ERROR;
    }
    bool has_position_state = false;
    bool has_velocity_state = false;
    for (const auto& state_if : joint.state_interfaces) {
        if (state_if.name == hardware_interface::HW_IF_POSITION) has_position_state = true;
        if (state_if.name == hardware_interface::HW_IF_VELOCITY) has_velocity_state = true;
    }
    if (!has_position_state || !has_velocity_state) {
        RCLCPP_FATAL(rclcpp::get_logger("DummyHardwareInterface"), "Joint '%s' must have state interfaces named '%s' AND '%s'. Check your URDF.",
                     joint.name.c_str(), hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "on_init successful");
  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> DummyHardwareInterface::export_state_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Exporting state interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
    // Keep exporting velocity state interface
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
  }

  RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "State interfaces exported successfully (%zu position, %zu velocity)", info_.joints.size(), info_.joints.size());
  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> DummyHardwareInterface::export_command_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Exporting command interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    // Assuming only position commands are needed
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
  }

  RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Command interfaces exported successfully (%zu position)", info_.joints.size());
  return command_interfaces;
}


hardware_interface::CallbackReturn DummyHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Activating hardware interface...");

  // --- Check if Serial Driver was initialized successfully ---
  // IoContext is a member, so it exists if the class exists. Check the driver pointer.
  if (!serial_driver_) {
       RCLCPP_FATAL(rclcpp::get_logger("DummyHardwareInterface"), "Serial driver not initialized. Cannot activate. Was on_init successful?");
       return hardware_interface::CallbackReturn::ERROR;
  }

  // --- Open Serial Port ---
  try {
      if (!serial_driver_->port()->is_open()) {
          RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Opening serial port: %s", serial_port_name_.c_str());
          serial_driver_->port()->open();
          std::this_thread::sleep_for(std::chrono::milliseconds(2500)); // Wait 2.5 seconds
          RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Serial port opened. Waiting for Arduino ready message...");

          // --- Wait for Arduino Ready Message ---
          const std::string ready_message = "INFO: Arduino Ready. Waiting for ROS commands.";
          const auto timeout = std::chrono::seconds(15); // Max wait time
          auto start_time = std::chrono::steady_clock::now();
          std::string line;
          bool ready_received = false;

          while (std::chrono::steady_clock::now() - start_time < timeout) {
              line = readSerialResponse();
              if (!line.empty()) {
                   RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Received from Arduino during init: %s", line.c_str());
                   if (line.find(ready_message) != std::string::npos) {
                       RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Arduino ready message detected.");
                       ready_received = true;
                       break;
                   }
              }
          }

          if (!ready_received) {
              RCLCPP_FATAL(rclcpp::get_logger("DummyHardwareInterface"), "Timeout waiting for Arduino ready message ('%s'). Check Arduino code and connection.", ready_message.c_str());
              try {
                  if (serial_driver_->port()->is_open()) serial_driver_->port()->close();
              } catch (const std::exception& close_ex) {
                    RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Exception closing port after activation failure: %s", close_ex.what());
              }
              return hardware_interface::CallbackReturn::ERROR;
          }
      } else {
           RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Serial port already open.");
      }
  } catch (const std::exception& e) {
      RCLCPP_FATAL(rclcpp::get_logger("DummyHardwareInterface"), "Failed to open serial port '%s' or wait for ready message: %s", serial_port_name_.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
  }

  // --- Initialize hardware states ---
  RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Setting initial hardware states...");
  for (uint i = 0; i < info_.joints.size(); i++)
  {
      const auto& joint_params = info_.joints[i].parameters;
      const std::string param_name = "initial_position";
      double initial_pos = 0.0;

      auto it = joint_params.find(param_name);
      if (it != joint_params.end()) {
          try {
              initial_pos = std::stod(it->second);
              RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Joint '%s': Using initial position from URDF parameter '%s': %f", info_.joints[i].name.c_str(), param_name.c_str(), initial_pos);
          } catch (const std::exception &e) { // Catch generic exception for stod
              RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Joint '%s': Error parsing URDF parameter '%s': '%s' (%s). Using default 0.0.", info_.joints[i].name.c_str(), param_name.c_str(), it->second.c_str(), e.what());
          }
      } else {
          RCLCPP_WARN(rclcpp::get_logger("DummyHardwareInterface"), "Joint '%s': URDF parameter '%s' not found. Defaulting initial position to 0.0.", info_.joints[i].name.c_str(), param_name.c_str());
      }

      hw_states_positions_[i] = initial_pos;
      hw_states_velocities_[i] = 0.0; // Initialize velocity to 0
      hw_commands_positions_[i] = initial_pos; // Command buffer starts at initial state
  }

  RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Hardware interface successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn DummyHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Deactivating hardware interface...");

  // --- Close Serial Port ---
  try {
      // Check if driver was successfully created AND port is open
      if (serial_driver_ && serial_driver_->port()->is_open()) {
          RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Closing serial port: %s", serial_port_name_.c_str());
          serial_driver_->port()->close();
          RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Serial port closed.");
      } else {
          RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Serial port already closed or driver not initialized.");
      }
  } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Exception closing serial port: %s", e.what());
  }

  // No need to explicitly destroy io_context_ (member variable)
  // serial_driver_ (unique_ptr) will be destroyed automatically when DummyHardwareInterface is destroyed

  RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Hardware interface successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}


// --- READING FROM ARDUINO ---
hardware_interface::return_type DummyHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) // Period not used
{
    // 1. Send Status Request Command ("S\n")
    if (!sendSerialCommand("S\n"))
    {
        RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "READ: Failed to send status request command ('S\\n').");
        return hardware_interface::return_type::ERROR;
    }

    // 2. Read Status Response ("S B S E G\n")
    std::string response_line = readSerialResponse();

    // 3. Basic Validation of Response
    if (response_line.empty())
    {
        RCLCPP_WARN(rclcpp::get_logger("DummyHardwareInterface"), "READ: Timeout or empty response received after status request.");
        return hardware_interface::return_type::ERROR; // Treat timeout as error
    }

    if (response_line.rfind("S ", 0) != 0)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("DummyHardwareInterface"),
            "READ: Received invalid status line format. Expected 'S ...\\n', Got: '%s'",
            response_line.c_str());
        return hardware_interface::return_type::ERROR;
    }

    // 4. Parse the Step Values from the Response String
    std::stringstream ss(response_line.substr(2));
    std::vector<int> current_steps(info_.joints.size());
    bool parse_ok = true;

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        if (!(ss >> current_steps[i]))
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("DummyHardwareInterface"),
                "READ: Failed to parse step value for joint index %zu from status response: '%s'",
                i, response_line.c_str());
            parse_ok = false;
            break;
        }
    }

    if (!parse_ok)
    {
        RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "READ: Aborting state update due to parsing error.");
        return hardware_interface::return_type::ERROR;
    }

    // --- Optional: Check for extra data ---
    std::string remaining_data;
    if (ss >> remaining_data) {
         RCLCPP_WARN(
            rclcpp::get_logger("DummyHardwareInterface"),
            "READ: Parsed expected values, but extra data remains: '%s'. Original line: '%s'",
            remaining_data.c_str(), response_line.c_str());
    }
    // --- End extra data check ---

    // 5. Update Hardware State Interfaces (Position and Velocity=0)
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        const std::string &joint_name = info_.joints[i].name;

        // --- Update Position State ---
        // ASSUMPTION: Gripper joint is named "left_finger_joint"
        if (joint_name == "left_finger_joint")
        {
            hw_states_positions_[i] = stepsToMeters(current_steps[i]);
        }
        else // Assuming arm joint
        {
            hw_states_positions_[i] = stepsToRadians(joint_name, current_steps[i]);
        }

        // --- Update Velocity State (Option 1: Set to Zero) ---
        hw_states_velocities_[i] = 0.0;

        RCLCPP_DEBUG(
            rclcpp::get_logger("DummyHardwareInterface"),
            "READ Joint '%s': Steps=%d -> PosState=%f rad/m, VelState=%f (Forced Zero)",
            joint_name.c_str(), current_steps[i], hw_states_positions_[i], hw_states_velocities_[i]);
    }

    return hardware_interface::return_type::OK;
}


// --- WRITING TO ARDUINO ---
hardware_interface::return_type DummyHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    std::stringstream command_stream;
    command_stream << "M:"; // Start MOVE command

    // ASSUMPTION: Joint order matches Arduino expectation (Base, Shoulder, Elbow, Gripper)
    for (size_t i = 0; i < hw_commands_positions_.size(); ++i) {
        int target_steps;
        const std::string& joint_name = info_.joints[i].name;

        // Check for NaN command
        if (std::isnan(hw_commands_positions_[i])) {
             RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "WRITE: Received NaN command for joint '%s'. Sending 0 steps instead.", joint_name.c_str());
             // Decide how to handle NaN. Sending 0 might be safer than erroring out.
             // Or return ERROR as before:
             // return hardware_interface::return_type::ERROR;
             target_steps = 0; // Send 0 steps as a fallback? Needs consideration.
             // A better approach might be to send the *current* position back if NaN is received.
             // Requires reading current state before calculating target_steps. Let's stick to erroring for now.
             RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "WRITE: Received NaN command for joint '%s'. Skipping write cycle.", joint_name.c_str());
             return hardware_interface::return_type::ERROR;
        }


        // Convert ROS command (radians/meters) to Arduino steps
        // ASSUMPTION: Gripper joint is named "left_finger_joint"
        if (joint_name == "left_finger_joint") {
             target_steps = metersToSteps(hw_commands_positions_[i]);
        } else {
             target_steps = radiansToSteps(joint_name, hw_commands_positions_[i]);
        }

        command_stream << target_steps;
        if (i < hw_commands_positions_.size() - 1) {
            command_stream << ",";
        }

        RCLCPP_DEBUG(rclcpp::get_logger("DummyHardwareInterface"), "WRITE Joint '%s': Command Pos=%f -> Steps=%d", joint_name.c_str(), hw_commands_positions_[i], target_steps);
    }
    command_stream << "\n"; // Terminate command

    // Send the command
    if (!sendSerialCommand(command_stream.str())) {
        RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "WRITE: Failed to send move command '%s' to Arduino.", command_stream.str().c_str());
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

} // namespace dummy_hw_interface

// Export class
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  dummy_hw_interface::DummyHardwareInterface, hardware_interface::SystemInterface)
// --- END OF FILE dummy_hw_interface.cpp ---