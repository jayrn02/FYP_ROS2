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
    // Increased overall timeout slightly, as individual reads might block longer now
    const auto read_timeout = std::chrono::milliseconds(200);
    const auto start_time = std::chrono::steady_clock::now();

    try {
        while (true) {
            // Check for overall timeout
            auto now = std::chrono::steady_clock::now();
            if (now - start_time > read_timeout) {
                RCLCPP_WARN(rclcpp::get_logger("DummyHardwareInterface"), "Timeout waiting for newline in serial response. Accumulated: '%s'", accumulated_response.c_str());
                return ""; // Timeout occurred before newline
            }

            // Try to receive a single byte. This might block based on port's configured timeout.
            size_t bytes_read = 0;
            try {
                 // Call receive without the timeout argument
                 bytes_read = serial_driver_->port()->receive(chunk);
            }
            // Removed the specific SerialPortTimeout catch block
            catch (const std::exception& e) {
                 // Handle other potential receive errors (e.g., port closed unexpectedly)
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
                // If receive returned 0 bytes, it likely means the port's internal timeout occurred
                // without receiving data. We simply let the loop continue and check our overall timeout.
                // Add a small sleep to prevent busy-waiting if the port timeout is very short or zero.
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

  // --- Initialize Clock ---
  try {
      clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  } catch (const std::exception & e) {
      RCLCPP_FATAL(rclcpp::get_logger("DummyHardwareInterface"), "Failed to create clock: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
  }

  // --- Read parameters ---
  try {
      serial_port_name_ = info_.hardware_parameters.at("serial_port");
      // We still read baud_rate, but won't use it in init_port for now
      serial_baud_rate_ = std::stoul(info_.hardware_parameters.at("baud_rate"));
      RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Serial Port: %s, Baud Rate (read): %u", serial_port_name_.c_str(), serial_baud_rate_);
  } catch (const std::out_of_range & ex) {
      RCLCPP_FATAL(rclcpp::get_logger("DummyHardwareInterface"), "Parameter 'serial_port' or 'baud_rate' not found in hardware info.");
      return hardware_interface::CallbackReturn::ERROR;
  } catch (const std::invalid_argument & ex) {
      RCLCPP_FATAL(rclcpp::get_logger("DummyHardwareInterface"), "Parameter 'baud_rate' is not a valid unsigned integer.");
      return hardware_interface::CallbackReturn::ERROR;
  }

  // --- Initialize Serial Driver ---
  try {
      // Initialize the SerialDriver (ensure io_context_ is available)
      serial_driver_ = std::make_unique<drivers::serial_driver::SerialDriver>(io_context_);

      // Initialize the port - simplified call
      // We assume configuration (baud, timeouts) might be set later or default.
      // This call might fail if init_port requires more arguments.
      serial_driver_->init_port(serial_port_name_);

      // TODO: Investigate how to set baud rate and timeouts for this serial_driver version.
      // It might involve calling methods on serial_driver_->port() after init_port
      // or passing more arguments to init_port if an overload exists.
      // Example (hypothetical):
      // serial_driver_->port()->set_baud_rate(serial_baud_rate_);
      // serial_driver_->port()->set_timeouts(...);

      RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Serial port initialized with name: %s. Baud rate/timeouts need verification.", serial_port_name_.c_str());

  } catch (const std::exception& e) {
       RCLCPP_FATAL(rclcpp::get_logger("DummyHardwareInterface"), "Failed during serial driver initialization: %s", e.what());
       return hardware_interface::CallbackReturn::ERROR;
  }


  // --- Resize and initialize storage vectors ---
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // --- Check interfaces ---
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Check if the joint has a command interface for position
    if (joint.command_interfaces.size() != 1) { /* ... error handling ... */ }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) { /* ... error handling ... */ }

    // Check if the joint has state interfaces for position and velocity
    bool has_pos_state = false;
    bool has_vel_state = false;
    for (const auto& state_if : joint.state_interfaces) {
        if (state_if.name == hardware_interface::HW_IF_POSITION) has_pos_state = true;
        if (state_if.name == hardware_interface::HW_IF_VELOCITY) has_vel_state = true;
    }
    if (!has_pos_state || !has_vel_state) { /* ... error handling ... */ }
  }

  RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "on_init successful (pending serial config verification)");
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
          RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Serial port opened successfully.");

          // --- Wait for Arduino to Initialize/Home ---
          int arduino_wait_seconds = 20; // Adjust this value if needed (e.g., 3 or 5)
          RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Waiting %d seconds for Arduino to initialize and home...", arduino_wait_seconds);
          std::this_thread::sleep_for(std::chrono::seconds(arduino_wait_seconds));
          RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Finished waiting for Arduino.");
          // --- End Wait ---

      } else {
           RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Serial port already open.");
      }
  } catch (const std::exception& e) {
      RCLCPP_FATAL(rclcpp::get_logger("DummyHardwareInterface"), "Failed to open serial port %s: %s", serial_port_name_.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
  }

  // --- Initialize hardware states ---
  // (Keep existing logic for initializing hw_states_positions_, hw_commands_positions_, hw_states_velocities_)
  // Option 1: Use initial values from URDF/xacro (as before)
  for (uint i = 0; i < hw_states_positions_.size(); i++)
  {
    // ... (existing initialization logic using initial_value or defaulting to 0) ...
     if (!info_.joints[i].state_interfaces[0].parameters.empty()) {
        double initial_pos = std::stod(info_.joints[i].state_interfaces[0].parameters.at("initial_value"));
        RCLCPP_INFO(rclcpp::get_logger("DummyHardwareInterface"), "Setting initial position for %s to %f", info_.joints[i].name.c_str(), initial_pos);
        hw_states_positions_[i] = initial_pos;
        hw_commands_positions_[i] = initial_pos; // Initialize command to initial position
    } else {
        RCLCPP_WARN(rclcpp::get_logger("DummyHardwareInterface"), "No initial value parameter found for %s position state. Defaulting to 0.", info_.joints[i].name.c_str());
        hw_states_positions_[i] = 0.0;
        hw_commands_positions_[i] = 0.0; // Initialize command to 0
    }
    hw_states_velocities_[i] = 0.0; // Initialize velocity to 0
  }
  // IMPORTANT: Consider if you want to perform an initial read *here* after waiting,
  // to synchronize the internal state with the Arduino's actual post-homing state
  // before controllers become fully active. This would involve calling sendSerialCommand("S\n")
  // and readSerialResponse() and parsing the result here. For now, we rely on the first
  // regular read() call after activation.


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
    // Check if serial port is open
    if (!serial_driver_ || !serial_driver_->port()->is_open()) {
         RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("DummyHardwareInterface"), *clock_, 1000, "Serial port not open in read().");
         return hardware_interface::return_type::ERROR;
    }

    const int MAX_READ_ATTEMPTS = 100; // Number of times to retry the request-response cycle
    const auto RETRY_DELAY = std::chrono::milliseconds(50); // Delay between retries

    for (int attempt = 1; attempt <= MAX_READ_ATTEMPTS; ++attempt)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("DummyHardwareInterface"), "Read attempt %d/%d", attempt, MAX_READ_ATTEMPTS);

        // --- Send Status Request Command ---
        if (!sendSerialCommand("S\n")) {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("DummyHardwareInterface"), *clock_, 1000, "Attempt %d: Failed to send status request 'S\\n'. Retrying...", attempt);
            std::this_thread::sleep_for(RETRY_DELAY); // Wait before next attempt
            continue; // Go to next iteration of the loop
        }

        // --- Optional: Short delay AFTER sending command ---
        // Give Arduino a moment to process 'S' before we try reading.
        // Adjust if needed (e.g., 20-50ms). If Arduino is fast, this might be 0.
        // std::this_thread::sleep_for(std::chrono::milliseconds(20));

        // --- Read the Expected Response (using helper with its own timeout) ---
        std::string response = readSerialResponse();

        if (response.empty()) {
            // Timeout occurred in readSerialResponse or other read error within it
            RCLCPP_WARN(rclcpp::get_logger("DummyHardwareInterface"), "Attempt %d: Did not receive response line within timeout after sending 'S\\n'. Retrying...", attempt);
            // No need for extra sleep here, readSerialResponse already waited
            continue; // Go to next iteration of the loop
        }

        // --- Check if the response is the expected status format ---
        if (response.rfind("S ", 0) == 0) { // Check for "S " prefix
             RCLCPP_DEBUG(rclcpp::get_logger("DummyHardwareInterface"), "Attempt %d: Received valid status response: '%s'", attempt, response.c_str());

             // --- PARSE THE VALID RESPONSE ---
             std::stringstream ss(response);
             std::string command_char; // Will be 'S'
             std::vector<int> steps;
             int temp_step;

             ss >> command_char; // Read the 'S'

             // Read the subsequent integer step values separated by spaces
             while (ss >> temp_step) {
                 steps.push_back(temp_step);
             }

             // Check if the correct number of step values were read
             if (steps.size() == info_.joints.size()) {
                 // Convert parsed steps to radians/meters and update state variables
                 for (size_t i = 0; i < info_.joints.size(); ++i) {
                     const std::string& joint_name = info_.joints[i].name;
                     if (joint_name == "left_finger_joint") { // Gripper
                         hw_states_positions_[i] = stepsToMeters(steps[i]);
                     } else { // Arm joints
                         hw_states_positions_[i] = stepsToRadians(joint_name, steps[i]);
                     }
                     hw_states_velocities_[i] = 0.0; // Keep velocity 0
                     RCLCPP_DEBUG(rclcpp::get_logger("DummyHardwareInterface"), "Parsed Joint %s: Steps=%d -> Pos State=%f", joint_name.c_str(), steps[i], hw_states_positions_[i]);
                 }
                 return hardware_interface::return_type::OK; // SUCCESS! Exit function.
             } else {
                 // Error if the number of parsed values doesn't match the expected number of joints
                 RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Attempt %d: Parsed %zu step values from valid status response, expected %zu. Response: '%s'. Retrying...", attempt, steps.size(), info_.joints.size(), response.c_str());
                 // Continue loop to retry
             }
             // --- END PARSING ---

        } else {
             // Received something, but not the expected "S ..." format
             RCLCPP_WARN(rclcpp::get_logger("DummyHardwareInterface"), "Attempt %d: Received unexpected response after 'S\\n': '%s'. Retrying...", attempt, response.c_str());
             // Continue loop to retry
        }

        // If we reach here, the attempt failed, wait before the next one
        if (attempt < MAX_READ_ATTEMPTS) {
             std::this_thread::sleep_for(RETRY_DELAY);
        }

    } // End for loop (retries)

    // If the loop finishes without returning OK, all attempts failed
    RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Failed to read valid status after %d attempts.", MAX_READ_ATTEMPTS);
    return hardware_interface::return_type::ERROR;
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