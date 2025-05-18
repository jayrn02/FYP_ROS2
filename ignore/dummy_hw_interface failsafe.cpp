#include "dummy_hw_interface/dummy_hw_interface.hpp"

#include <vector>
#include <string>
#include <limits>
#include <chrono> // For timeouts/sleeps
#include <thread> // For sleeps
#include <stdexcept> // For exceptions
#include <sstream> // For string manipulation

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp" // Make sure this is included
#include "io_context/io_context.hpp"       // Include IoContext header

// Use drivers namespace for convenience
using namespace drivers::serial_driver;

namespace dummy_hw_interface
{

// Helper function implementations (to be refined based on Arduino protocol)
// --- Start Helper Functions ---

// Converts radians (ROS) to steps (Arduino) for arm joints
int DummyHardwareInterface::radiansToSteps(const std::string& joint_name, double radians)
{
    int steps = 0;
    if (joint_name == "base_link_to_upper_base_link") {
        steps = static_cast<int>(radians * ARM_STEPS_PER_RADIAN) + BASE_HOME_STEPS;
    } else if (joint_name == "upper_base_link_to_lower_link") {
        steps = static_cast<int>(radians * ARM_STEPS_PER_RADIAN) + SHOULDER_HOME_STEPS;
    } else if (joint_name == "lower_link_to_upper_link") {
        steps = static_cast<int>(radians * ARM_STEPS_PER_RADIAN) + ELBOW_HOME_STEPS;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Unknown arm joint name for radiansToSteps: %s", joint_name.c_str());
    }
    // Add clamping/validation if necessary
    return steps;
}

// Converts steps (Arduino) to radians (ROS) for arm joints
double DummyHardwareInterface::stepsToRadians(const std::string& joint_name, int steps)
{
    double radians = 0.0;
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
    // Clamp meters to valid range [0, GRIPPER_MAX_METERS]
    meters = std::max(0.0, std::min(meters, GRIPPER_MAX_METERS));
    int steps = static_cast<int>(meters * GRIPPER_STEPS_PER_METER);
    // Clamp steps to valid range [0, GRIPPER_MAX_STEPS]
    steps = std::max(0, std::min(steps, GRIPPER_MAX_STEPS));
    return steps;
}

// Converts steps (Arduino) to meters (ROS) for gripper joint
double DummyHardwareInterface::stepsToMeters(int steps)
{
    // Clamp steps to valid range [0, GRIPPER_MAX_STEPS]
    steps = std::max(0, std::min(steps, GRIPPER_MAX_STEPS));
    double meters = static_cast<double>(steps) / GRIPPER_STEPS_PER_METER;
    // Clamp meters to valid range [0, GRIPPER_MAX_METERS]
    meters = std::max(0.0, std::min(meters, GRIPPER_MAX_METERS));
    return meters;
}


bool DummyHardwareInterface::sendSerialCommand(const std::string& command)
{
    if (!serial_driver_ || !serial_driver_->port()->is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Serial port not open, cannot send command.");
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
        RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Serial port not open, cannot read response.");
        return "";
    }

    std::string accumulated_response = "";
    std::vector<uint8_t> chunk(1); // Read one byte at a time
    // Define a timeout for the read operation
    const auto read_timeout = std::chrono::milliseconds(500); // Adjust timeout as needed
    const auto start_time = std::chrono::steady_clock::now();

    try {
        while (true) {
            // Check for overall timeout
            auto now = std::chrono::steady_clock::now();
            if (now - start_time > read_timeout) {
                RCLCPP_WARN(rclcpp::get_logger("DummyHardwareInterface"), "Timeout waiting for newline in serial response. Accumulated: '%s'", accumulated_response.c_str());
                return ""; // Timeout occurred before newline
            }

            // Try to receive a single byte.
            size_t bytes_read = 0;
            try {
                 // Call receive without a specific timeout argument, relying on port's config or overall loop timeout
                 bytes_read = serial_driver_->port()->receive(chunk);
            }
            catch (const std::exception& e) {
                 // Handle potential receive errors (e.g., port closed unexpectedly)
                 RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Error during serial receive: %s", e.what());
                 return ""; // Error during read
            }

            if (bytes_read > 0) {
                char received_char = static_cast<char>(chunk[0]);
                accumulated_response += received_char;

                // Check if we received the newline character
                if (received_char == '\n') {
                    RCLCPP_DEBUG(rclcpp::get_logger("DummyHardwareInterface"), "Received complete line: %s", accumulated_response.c_str());

                    // Remove trailing newline and potential carriage return
                    accumulated_response.pop_back(); // Remove '\n'
                    if (!accumulated_response.empty() && accumulated_response.back() == '\r') {
                        accumulated_response.pop_back(); // Remove '\r'
                    }
                    RCLCPP_DEBUG(rclcpp::get_logger("DummyHardwareInterface"), "Processed response line: %s", accumulated_response.c_str());
                    return accumulated_response; // Success! Return the complete line.
                }
            } else {
                // If receive returned 0 bytes, it might mean the port's internal timeout occurred
                // without receiving data. Add a small sleep to prevent busy-waiting.
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            // If newline not found yet, the loop continues (checking overall timeout)
        }
    } catch (const std::exception& e) { // Catch potential errors from steady_clock or string ops
        RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Exception in readSerialResponse loop: %s", e.what());
        return "";
    }
}

// --- End Helper Functions ---


hardware_interface::CallbackReturn DummyHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "on_init starting");

  // --- Read parameters from hardware info (xacro) ---
  try {
      serial_port_name_ = info_.hardware_parameters.at("serial_port");
      serial_baud_rate_ = std::stoul(info_.hardware_parameters.at("baud_rate")); // Use stoul for unsigned long
      RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Serial Port: %s, Baud Rate: %u", serial_port_name_.c_str(), serial_baud_rate_);
  } catch (const std::out_of_range & ex) {
      RCLCPP_FATAL(rclcpp::get_logger("DummyHardwareInterface"), "Parameters 'serial_port' or 'baud_rate' not found in URDF/xacro hardware parameters.");
      return hardware_interface::CallbackReturn::ERROR;
  } catch (const std::invalid_argument & ex) {
      RCLCPP_FATAL(rclcpp::get_logger("DummyHardwareInterface"), "Parameter 'baud_rate' is not a valid unsigned integer.");
      return hardware_interface::CallbackReturn::ERROR;
  }

  // --- Initialize Serial Driver ---
  try {
      // Use default SerialPortConfig for now (8N1, no flow control)
      auto config = std::make_shared<drivers::serial_driver::SerialPortConfig>(serial_baud_rate_,
                                                                               drivers::serial_driver::FlowControl::NONE,
                                                                               drivers::serial_driver::Parity::NONE,
                                                                               drivers::serial_driver::StopBits::ONE);
      // Pass io_context_ to the constructor
      serial_driver_ = std::make_unique<drivers::serial_driver::SerialDriver>(io_context_);
      // Set device name and config after creating the object
      serial_driver_->init_port(serial_port_name_, *config);
      // Note: Port is not opened here, but in on_activate
      RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Serial driver initialized for port %s.", serial_port_name_.c_str());
  } catch (const std::exception& e) {
       RCLCPP_FATAL(rclcpp::get_logger("DummyHardwareInterface"), "Failed during serial driver initialization: %s", e.what());
       return hardware_interface::CallbackReturn::ERROR;
  }

  // --- Resize and initialize storage vectors ---
  // Use the member variables declared in the header
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // --- Check interfaces (same as before) ---
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Command interface check
    if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(rclcpp::get_logger("DummyHardwareInterface"), "Joint '%s' has incorrect command interfaces.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    // State interface check
    if (joint.state_interfaces.size() != 2 ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("DummyHardwareInterface"), "Joint '%s' has incorrect state interfaces.", joint.name.c_str());
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
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i])); // Use member variable
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i])); // Use member variable
  }

  RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "State interfaces exported successfully");
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DummyHardwareInterface::export_command_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Exporting command interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i])); // Use member variable
  }

  RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Command interfaces exported successfully");
  return command_interfaces;
}

hardware_interface::CallbackReturn DummyHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Activating ...please wait...");

  // --- Open Serial Port ---
  try {
      if (!serial_driver_->port()->is_open()) {
          serial_driver_->port()->open();
          // Add a small delay to allow Arduino to reset if needed after serial connection
          // Increased delay slightly to give Arduino more time before we start reading
          std::this_thread::sleep_for(std::chrono::milliseconds(2500)); // Wait 2.5 seconds
          RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Serial port opened. Waiting for Arduino ready message...");

          // *** ADDED: Wait for Arduino Ready Message ***
          const std::string ready_message = "INFO: Arduino Ready. Waiting for ROS commands.";
          const auto timeout = std::chrono::seconds(15); // Max wait time for Arduino ready
          auto start_time = std::chrono::steady_clock::now();
          std::string line;
          bool ready_received = false;

          while (std::chrono::steady_clock::now() - start_time < timeout) {
              line = readSerialResponse(); // Use existing read function
              if (!line.empty()) {
                   RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Received from Arduino during init: %s", line.c_str());
                   // Check if the received line *contains* the ready message
                   if (line.find(ready_message) != std::string::npos) {
                       RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Arduino ready message received.");
                       ready_received = true;
                       break; // Exit the loop once ready message is found
                   }
              } else {
                  // Small delay if readSerialResponse timed out internally or returned empty
                  std::this_thread::sleep_for(std::chrono::milliseconds(50));
              }
          }

          if (!ready_received) {
              RCLCPP_FATAL(rclcpp::get_logger("DummyHardwareInterface"), "Timeout waiting for Arduino ready message ('%s').", ready_message.c_str());
              // Attempt to close port before returning error
              if (serial_driver_->port()->is_open()) serial_driver_->port()->close();
              return hardware_interface::CallbackReturn::ERROR;
          }
          // *** END ADDED SECTION ***

      }
  } catch (const std::exception& e) {
      RCLCPP_FATAL(rclcpp::get_logger("DummyHardwareInterface"), "Failed to open serial port %s or wait for ready: %s", serial_port_name_.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
  }

  // --- Initialize hardware states ---
  // Set initial states safely, checking for URDF parameters.
  // This part runs *after* the ready message is confirmed.
  for (uint i = 0; i < hw_states_positions_.size(); i++)
  {
      const auto& joint_params = info_.joints[i].parameters;
      const std::string param_name = "initial_position"; // Parameter name we are looking for

      // Use find() to safely check if the parameter exists
      auto it = joint_params.find(param_name);
      if (it != joint_params.end()) {
          // Parameter found, convert it to double
          try {
              hw_states_positions_[i] = std::stod(it->second);
              RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Setting initial position for %s from URDF parameter '%s' to %f", info_.joints[i].name.c_str(), param_name.c_str(), hw_states_positions_[i]);
          } catch (const std::invalid_argument &e) {
              RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Invalid number format for URDF parameter '%s' for joint %s: %s. Defaulting to 0.0.", param_name.c_str(), info_.joints[i].name.c_str(), it->second.c_str());
              hw_states_positions_[i] = 0.0;
          } catch (const std::out_of_range &e) {
               RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Out of range error for URDF parameter '%s' for joint %s: %s. Defaulting to 0.0.", param_name.c_str(), info_.joints[i].name.c_str(), it->second.c_str());
               hw_states_positions_[i] = 0.0;
          }
      } else {
          // Parameter NOT found, use a default value and log a warning
          RCLCPP_WARN(rclcpp::get_logger("DummyHardwareInterface"), "URDF parameter '%s' not found for joint %s. Defaulting initial position to 0.0.", param_name.c_str(), info_.joints[i].name.c_str());
          hw_states_positions_[i] = 0.0;
      }

      // Initialize velocity and command regardless of parameter presence
      hw_states_velocities_[i] = 0.0; // Assuming starting velocity is 0
      hw_commands_positions_[i] = hw_states_positions_[i]; // Initialize commands to initial state
  }


  RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DummyHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Deactivating ...please wait...");

  // --- Close Serial Port ---
  try {
      if (serial_driver_ && serial_driver_->port()->is_open()) {
          serial_driver_->port()->close();
          RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Serial port closed.");
      }
  } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Error closing serial port: %s", e.what());
      // Continue deactivation even if closing fails
  }

  RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DummyHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // --- Read current state from Arduino ---
    // 1. Send 'S\n' command.
    // 2. Read lines until a valid status response ("S ...") is found or retries expire.
    // 3. Parse the response.
    // 4. Convert steps to radians/meters.
    // 5. Update hw_states_positions_.

    if (!sendSerialCommand("S\n")) { // Send status request
        RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Failed to send status request 'S\\n' to Arduino.");
        return hardware_interface::return_type::ERROR; // Indicate read error
    }

    std::string response = "";
    bool valid_response_found = false;
    const int max_read_attempts = 5; // Try reading up to 5 lines before giving up
    int read_attempts = 0;

    while (read_attempts < max_read_attempts) {
        read_attempts++;
        response = readSerialResponse(); // Read one line using helper

        if (response.empty()) {
            // Timeout or empty read from readSerialResponse()
            RCLCPP_WARN(rclcpp::get_logger("DummyHardwareInterface"), "Read attempt %d/%d: No response line received from Arduino.", read_attempts, max_read_attempts);
            // Add a small delay before trying to read again
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue; // Try reading again
        }

        // Check if the response starts with 'S' and a space
        if (response.rfind("S ", 0) == 0) {
             RCLCPP_DEBUG(rclcpp::get_logger("DummyHardwareInterface"), "Read attempt %d/%d: Found valid status line: '%s'", read_attempts, max_read_attempts, response.c_str());
             valid_response_found = true;
             break; // Exit the loop, we have a valid line
        } else {
             // Discard invalid/unexpected line and log it
             RCLCPP_DEBUG(rclcpp::get_logger("DummyHardwareInterface"), "Read attempt %d/%d: Discarding unexpected line: '%s'", read_attempts, max_read_attempts, response.c_str());
             // Continue loop to read the next line
        }
    } // End while loop

    // Check if we exited the loop because we found a valid response or hit the limit
    if (!valid_response_found) {
        RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Failed to find valid status line ('S ...') after %d read attempts.", max_read_attempts);
        return hardware_interface::return_type::ERROR;
    }
    // --- End Read Until Valid ---


    // --- PARSE THE VALID RESPONSE (response variable now holds the 'S ...' line) ---
    std::string values_str = response.substr(2); // Get the part after "S "
    std::stringstream ss(values_str);
    std::vector<int> steps;
    int step_value;

    // Read space-separated integer values
    while (ss >> step_value) {
        steps.push_back(step_value);
    }

    // Check if the stream failed for reasons other than reaching the end
    if (!ss.eof() && ss.fail()) {
         RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Error parsing step values in response: '%s'. Invalid number format?", values_str.c_str());
         return hardware_interface::return_type::ERROR;
    }

    // Check if the correct number of step values were read
    if (steps.size() == info_.joints.size()) {
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            const std::string& joint_name = info_.joints[i].name;
            int current_step_value = steps[i]; // Direct mapping based on order

            // Assign based on the known order: Base, Shoulder, Elbow, Gripper
            if (joint_name == "base_link_to_upper_base_link") {
                hw_states_positions_[i] = stepsToRadians(joint_name, steps[0]); // Base
            } else if (joint_name == "upper_base_link_to_lower_link") {
                hw_states_positions_[i] = stepsToRadians(joint_name, steps[1]); // Shoulder
            } else if (joint_name == "lower_link_to_upper_link") {
                hw_states_positions_[i] = stepsToRadians(joint_name, steps[2]); // Elbow
            } else if (joint_name == "left_finger_joint") { // Assuming this is the gripper
                hw_states_positions_[i] = stepsToMeters(steps[3]); // Gripper
            } else {
                 RCLCPP_WARN(rclcpp::get_logger("DummyHardwareInterface"), "Unknown joint name '%s' during read state update.", joint_name.c_str());
                 continue;
            }

            hw_states_velocities_[i] = 0.0; // Keep velocity 0 for now
            RCLCPP_DEBUG(rclcpp::get_logger("DummyHardwareInterface"), "Read Joint %s: Steps=%d -> Pos State=%f", joint_name.c_str(), current_step_value, hw_states_positions_[i]);
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Parsed %zu step values from response '%s', expected %zu.", steps.size(), response.c_str(), info_.joints.size());
        return hardware_interface::return_type::ERROR;
    }
    // --- END PARSING ---

    return hardware_interface::return_type::OK; // Indicate successful read and parse
}

hardware_interface::return_type DummyHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // --- Write commands to Arduino ---
    // 1. Iterate through hw_commands_positions_
    // 2. Convert radians/meters to steps using helper functions
    // 3. Format the command string (e.g., "M:stepsB,stepsS,stepsE,stepsG\n")
    // 4. Send the command string via serial

    // --- Placeholder Implementation ---
    std::stringstream command_stream;
    command_stream << "M:"; // Replace "M:" with your Arduino move command prefix

    for (size_t i = 0; i < hw_commands_positions_.size(); ++i) {
        int target_steps;
        const std::string& joint_name = info_.joints[i].name;

        if (joint_name == "left_finger_joint") { // Assuming gripper
             target_steps = metersToSteps(hw_commands_positions_[i]);
        } else { // Assuming arm joint
             target_steps = radiansToSteps(joint_name, hw_commands_positions_[i]);
        }

        command_stream << target_steps;
        if (i < hw_commands_positions_.size() - 1) {
            command_stream << ","; // Add comma separator
        }
         RCLCPP_DEBUG(rclcpp::get_logger("DummyHardwareInterface"), "Write Joint %s: Command Pos=%f -> Steps=%d", joint_name.c_str(), hw_commands_positions_[i], target_steps);
    }
    command_stream << "\n"; // Add newline termination

    if (!sendSerialCommand(command_stream.str())) {
        RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Failed to send move command to Arduino.");
        return hardware_interface::return_type::ERROR; // Indicate write error
    }
    // --- End Placeholder ---

    return hardware_interface::return_type::OK;
}

} // namespace dummy_hw_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  dummy_hw_interface::DummyHardwareInterface, hardware_interface::SystemInterface)