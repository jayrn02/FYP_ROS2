#include "arduino_hw_interface/arduino_hw_interface.hpp"

#include <vector>
#include <string>
#include <limits>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <sstream>
#include <algorithm> // For std::find

#include "hardware_interface/types/hardware_interface_type_values.hpp"

// Use drivers namespace for convenience
using namespace drivers::serial_driver;
using drivers::common::IoContext;

namespace arduino_hw_interface
{

// --- Helper: String trim functions ---
std::string trim_right(const std::string& s, const std::string& delimiters = " \f\n\r\t\v") {
    return s.substr(0, s.find_last_not_of(delimiters) + 1);
}
std::string trim_left(const std::string& s, const std::string& delimiters = " \f\n\r\t\v") {
    return s.substr(s.find_first_not_of(delimiters));
}
std::string trim(const std::string& s, const std::string& delimiters = " \f\n\r\t\v") {
    return trim_left(trim_right(s, delimiters), delimiters);
}


// --- Conversion Helpers ---
int ArduinoHardwareInterface::rosToArduinoSteps(const std::string& joint_name, double ros_value)
{
    int steps = 0;
    if (joint_name == joint_names_in_arduino_order_[0]) { // Base
        steps = static_cast<int>(ros_value * ARM_STEPS_PER_RADIAN) + BASE_HOME_STEPS_OFFSET;
    } else if (joint_name == joint_names_in_arduino_order_[1]) { // Shoulder
        steps = static_cast<int>(ros_value * ARM_STEPS_PER_RADIAN) + SHOULDER_HOME_STEPS_OFFSET;
    } else if (joint_name == joint_names_in_arduino_order_[2]) { // Elbow
        steps = static_cast<int>(ros_value * ARM_STEPS_PER_RADIAN) + ELBOW_HOME_STEPS_OFFSET;
    } else if (joint_name == joint_names_in_arduino_order_[3]) { // Gripper
        // Ensure gripper value is within 0 to GRIPPER_MAX_METERS (implicitly from URDF)
        // double constrained_meters = std::max(0.0, std::min(ros_value, GRIPPER_MAX_METERS)); 
        // The above GRIPPER_MAX_METERS is not defined, rely on controller limits.
        steps = static_cast<int>(ros_value * GRIPPER_STEPS_PER_METER);
        steps = std::max(0, std::min(steps, GRIPPER_MAX_STEPS));
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("ArduinoHardwareInterface"), "Unknown joint name for rosToArduinoSteps: %s", joint_name.c_str());
    }
    return steps;
}

double ArduinoHardwareInterface::arduinoToRosUnits(const std::string& joint_name, int arduino_steps)
{
    double ros_value = 0.0;
    if (joint_name == joint_names_in_arduino_order_[0]) { // Base
        ros_value = static_cast<double>(arduino_steps - BASE_HOME_STEPS_OFFSET) / ARM_STEPS_PER_RADIAN;
    } else if (joint_name == joint_names_in_arduino_order_[1]) { // Shoulder
        ros_value = static_cast<double>(arduino_steps - SHOULDER_HOME_STEPS_OFFSET) / ARM_STEPS_PER_RADIAN;
    } else if (joint_name == joint_names_in_arduino_order_[2]) { // Elbow
        ros_value = static_cast<double>(arduino_steps - ELBOW_HOME_STEPS_OFFSET) / ARM_STEPS_PER_RADIAN;
    } else if (joint_name == joint_names_in_arduino_order_[3]) { // Gripper
        arduino_steps = std::max(0, std::min(arduino_steps, GRIPPER_MAX_STEPS));
        ros_value = static_cast<double>(arduino_steps) / GRIPPER_STEPS_PER_METER;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("ArduinoHardwareInterface"), "Unknown joint name for arduinoToRosUnits: %s", joint_name.c_str());
    }
    return ros_value;
}

// --- Serial Communication Helpers ---
bool ArduinoHardwareInterface::sendSerialCommand(const std::string& command)
{
    if (!serial_driver_ || !serial_driver_->port()->is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("ArduinoHardwareInterface"), "Serial port not open or driver not initialized. Cannot send command: %s", command.c_str());
        return false;
    }
    try {
        RCLCPP_DEBUG(rclcpp::get_logger("ArduinoHardwareInterface"), "Sending command: %s", command.c_str());
        std::vector<uint8_t> data(command.begin(), command.end());
        serial_driver_->port()->send(data);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ArduinoHardwareInterface"), "Error sending serial command '%s': %s", command.c_str(), e.what());
        return false;
    }
}

std::string ArduinoHardwareInterface::receiveSerialLine(std::chrono::milliseconds timeout)
{
    if (!serial_driver_ || !serial_driver_->port()->is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("ArduinoHardwareInterface"), "Serial port not open, cannot read.");
        return "";
    }
    std::string line_buffer;
    line_buffer.reserve(128); // Pre-allocate some space
    auto start_time = std::chrono::steady_clock::now();

    try {
        while (true) {
            if (std::chrono::steady_clock::now() - start_time > timeout) {
                RCLCPP_WARN(rclcpp::get_logger("ArduinoHardwareInterface"), "Timeout waiting for serial line. Accumulated: '%s'", line_buffer.c_str());
                return ""; 
            }

            std::vector<uint8_t> chunk(1);
            size_t bytes_read = serial_driver_->port()->receive(chunk);

            if (bytes_read > 0) {
                char c = static_cast<char>(chunk[0]);
                if (c == '\n') {
                    RCLCPP_DEBUG(rclcpp::get_logger("ArduinoHardwareInterface"), "Received line: %s", trim(line_buffer).c_str());
                    return trim(line_buffer);
                } else if (c == '\r') {
                    // Ignore carriage return if it's not part of a CR+LF sequence
                } else if (isprint(c) || isspace(c)) { // Accept printable chars and spaces
                     line_buffer += c;
                     if (line_buffer.length() > 256) { // Safety break for extremely long lines
                        RCLCPP_ERROR(rclcpp::get_logger("ArduinoHardwareInterface"), "Serial line buffer overflow. Line too long.");
                        return "";
                     }
                }
                // Reset timeout if we are actively receiving characters for the current line
                start_time = std::chrono::steady_clock::now(); 
            } else {
                // No data, brief pause to prevent busy loop if port()->receive is non-blocking with 0 bytes
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ArduinoHardwareInterface"), "Exception during serial read: %s. Accumulated: '%s'", e.what(), line_buffer.c_str());
        return "";
    }
}


// --- ros2_control Interface Methods ---
hardware_interface::CallbackReturn ArduinoHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "on_init starting for Arduino Hardware Interface");

    // --- Read hardware parameters ---
    try {
        serial_port_name_ = info_.hardware_parameters.at("serial_port");
        serial_baud_rate_ = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("baud_rate")));
        // Optional: Read timeouts if defined in URDF, otherwise use defaults
        if (info_.hardware_parameters.count("serial_read_timeout_ms")) {
            serial_read_timeout_ = std::chrono::milliseconds(std::stoul(info_.hardware_parameters.at("serial_read_timeout_ms")));
        }
        if (info_.hardware_parameters.count("arduino_ready_timeout_ms")) {
            arduino_ready_timeout_ = std::chrono::milliseconds(std::stoul(info_.hardware_parameters.at("arduino_ready_timeout_ms")));
        }
        if (info_.hardware_parameters.count("post_open_delay_ms")) {
            post_open_delay_ = std::chrono::milliseconds(std::stoul(info_.hardware_parameters.at("post_open_delay_ms")));
        }
        RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "Serial Port: %s, Baud: %u, ReadTimeout: %ldms, ReadyTimeout: %ldms, PostOpenDelay: %ldms",
                    serial_port_name_.c_str(), serial_baud_rate_, serial_read_timeout_.count(), arduino_ready_timeout_.count(), post_open_delay_.count());
    } catch (const std::out_of_range& ex) {
        RCLCPP_FATAL(rclcpp::get_logger("ArduinoHardwareInterface"), "Missing required hardware parameter: %s", ex.what());
        return hardware_interface::CallbackReturn::ERROR;
    } catch (const std::exception& ex) {
        RCLCPP_FATAL(rclcpp::get_logger("ArduinoHardwareInterface"), "Error parsing hardware parameters: %s", ex.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // --- Initialize Serial Driver ---
    try {
        auto config = std::make_shared<drivers::serial_driver::SerialPortConfig>(serial_baud_rate_, FlowControl::NONE, Parity::NONE, StopBits::ONE);
        serial_driver_ = std::make_unique<drivers::serial_driver::SerialDriver>(io_context_);
        serial_driver_->init_port(serial_port_name_, *config);
        RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "Serial driver initialized for port %s.", serial_port_name_.c_str());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("ArduinoHardwareInterface"), "Failed during serial driver setup: %s", e.what());
        serial_driver_.reset();
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    // --- Setup joint structures ---
    // Expected order from Arduino: Base, Shoulder, Elbow, Gripper
    // URDF names: base_link_to_upper_base_link, upper_base_link_to_lower_link, lower_link_to_upper_link, left_finger_joint
    const std::vector<std::string> expected_joint_order = {
        "base_link_to_upper_base_link", 
        "upper_base_link_to_lower_link", 
        "lower_link_to_upper_link", 
        "left_finger_joint"
    };
    
    if (info_.joints.size() != expected_joint_order.size()) {
        RCLCPP_FATAL(rclcpp::get_logger("ArduinoHardwareInterface"), "Expected %zu joints in URDF, but found %zu.", expected_joint_order.size(), info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    joint_names_in_arduino_order_.resize(info_.joints.size());
    hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    prev_hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const auto& joint_info : info_.joints) {
        auto it = std::find(expected_joint_order.begin(), expected_joint_order.end(), joint_info.name);
        if (it == expected_joint_order.end()) {
            RCLCPP_FATAL(rclcpp::get_logger("ArduinoHardwareInterface"), "URDF joint '%s' is not one of the expected controllable joints.", joint_info.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        size_t arduino_idx = std::distance(expected_joint_order.begin(), it);
        joint_name_to_index_[joint_info.name] = arduino_idx; // Store URDF name to its Arduino order index
        joint_names_in_arduino_order_[arduino_idx] = joint_info.name; // Store URDF name at its Arduino order index

        RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "Mapped URDF joint '%s' to Arduino index %zu", joint_info.name.c_str(), arduino_idx);
    }
    
    // Verify all expected joints were found and mapped
    for(size_t i=0; i < expected_joint_order.size(); ++i) {
        if (joint_names_in_arduino_order_[i].empty()) {
             RCLCPP_FATAL(rclcpp::get_logger("ArduinoHardwareInterface"), "Expected URDF joint '%s' for Arduino index %zu was not found in hardware info.", expected_joint_order[i].c_str(), i);
             return hardware_interface::CallbackReturn::ERROR;
        }
    }


    // --- Validate Joint Interfaces ---
    for (const hardware_interface::ComponentInfo & joint : info_.joints) {
        if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("ArduinoHardwareInterface"), "Joint '%s' must have exactly one command interface named '%s'.", joint.name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (!(joint.state_interfaces.size() == 2 && 
              joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION &&
              joint.state_interfaces[1].name == hardware_interface::HW_IF_VELOCITY) &&
            !(joint.state_interfaces.size() == 2 && 
              joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY &&
              joint.state_interfaces[1].name == hardware_interface::HW_IF_POSITION)) { // Order can vary
            RCLCPP_FATAL(rclcpp::get_logger("ArduinoHardwareInterface"), "Joint '%s' must have exactly two state interfaces: '%s' and '%s'.", joint.name.c_str(), hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "on_init successful");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArduinoHardwareInterface::export_state_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "Exporting state interfaces");
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        // Use the URDF joint name from info_.joints[i].name
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArduinoHardwareInterface::export_command_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "Exporting command interfaces");
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
         // Use the URDF joint name from info_.joints[i].name
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn ArduinoHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "Activating hardware interface...");
    if (!serial_driver_) {
        RCLCPP_FATAL(rclcpp::get_logger("ArduinoHardwareInterface"), "Serial driver not initialized. Cannot activate.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    try {
        if (!serial_driver_->port()->is_open()) {
            RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "Opening serial port: %s", serial_port_name_.c_str());
            serial_driver_->port()->open();
            RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "Applying post-open delay of %ld ms...", post_open_delay_.count());
            std::this_thread::sleep_for(post_open_delay_);
            RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "Serial port opened. Waiting for Arduino ready message...");

            const std::string ready_message_substring = "Arduino Ready";
            auto start_time = std::chrono::steady_clock::now();
            std::string line;
            bool ready_received = false;
            while (std::chrono::steady_clock::now() - start_time < arduino_ready_timeout_) {
                line = receiveSerialLine(serial_read_timeout_); // Use combined timeout logic
                if (!line.empty()) {
                    RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "From Arduino during activation: %s", line.c_str());
                    if (line.find(ready_message_substring) != std::string::npos) {
                        RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "Arduino ready message detected.");
                        ready_received = true;
                        break;
                    }
                }
            }
            if (!ready_received) {
                RCLCPP_FATAL(rclcpp::get_logger("ArduinoHardwareInterface"), "Timeout waiting for Arduino ready message (containing '%s'). Check Arduino and connection.", ready_message_substring.c_str());
                if (serial_driver_->port()->is_open()) serial_driver_->port()->close();
                return hardware_interface::CallbackReturn::ERROR;
            }
        } else {
            RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "Serial port already open.");
        }
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("ArduinoHardwareInterface"), "Failed to open serial port '%s' or wait for ready: %s", serial_port_name_.c_str(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize hardware states from URDF initial_values or to 0.0
    RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "Setting initial hardware states...");
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const auto& joint_info = info_.joints[i];
        double initial_pos = 0.0;
        auto it_param = joint_info.parameters.find("initial_position"); // As per ros2_control xacro
         if (it_param != joint_info.parameters.end()) {
            try {
                initial_pos = std::stod(it_param->second);
                RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "Joint '%s': Using initial position from URDF: %f", joint_info.name.c_str(), initial_pos);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(rclcpp::get_logger("ArduinoHardwareInterface"), "Joint '%s': Error parsing initial_position '%s': %s. Using 0.0.", joint_info.name.c_str(), it_param->second.c_str(), e.what());
            }
        } else {
             RCLCPP_WARN(rclcpp::get_logger("ArduinoHardwareInterface"), "Joint '%s': initial_position parameter not found in URDF. Using 0.0.", joint_info.name.c_str());
        }
        
        hw_states_positions_[i] = initial_pos;
        prev_hw_states_positions_[i] = initial_pos; // Initialize previous state for velocity calc
        hw_states_velocities_[i] = 0.0;
        hw_commands_positions_[i] = initial_pos; // Command buffer starts at initial state
    }
    
    // Optional: Perform an initial read to sync with Arduino state after it's ready
    // This might be good if Arduino doesn't necessarily start at the URDF initial positions.
    // For now, we assume URDF initial positions are the source of truth at activation.

    RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "Hardware interface successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArduinoHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "Deactivating hardware interface...");
    try {
        if (serial_driver_ && serial_driver_->port()->is_open()) {
            RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "Closing serial port: %s", serial_port_name_.c_str());
            serial_driver_->port()->close();
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ArduinoHardwareInterface"), "Exception closing serial port: %s", e.what());
    }
    RCLCPP_INFO(rclcpp::get_logger("ArduinoHardwareInterface"), "Hardware interface successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArduinoHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    if (!sendSerialCommand("S\n")) {
        RCLCPP_ERROR(rclcpp::get_logger("ArduinoHardwareInterface"), "READ: Failed to send status request ('S\n').");
        return hardware_interface::return_type::ERROR;
    }

    std::string response_line;
    bool status_received = false;
    auto read_start_time = std::chrono::steady_clock::now();

    // Try to read until we get the 'S ' status line or global timeout for read op
    while(std::chrono::steady_clock::now() - read_start_time < std::chrono::milliseconds(serial_read_timeout_.count() * 2) ) { // Give a bit more time overall
        response_line = receiveSerialLine(serial_read_timeout_);
        if (response_line.empty()) { // Timeout from receiveSerialLine or error
            // If it was a genuine timeout from receiveSerialLine, it would have logged.
            // If it's empty due to other error, also logged.
            // We might want to break or continue based on specific needs.
            // For now, if receiveSerialLine returns empty, we assume a significant read issue for this cycle.
            RCLCPP_WARN(rclcpp::get_logger("ArduinoHardwareInterface"), "READ: Did not receive any line from Arduino in allowed time slot.");
            return hardware_interface::return_type::ERROR; 
        }

        if (response_line.rfind("S ", 0) == 0) { // Check if it's the status line
            status_received = true;
            break; 
        } else {
            RCLCPP_DEBUG(rclcpp::get_logger("ArduinoHardwareInterface"), "READ: Discarding non-status line from Arduino: '%s'", response_line.c_str());
            // Loop again to get the next line
        }
    }
    
    if (!status_received) {
         RCLCPP_ERROR(rclcpp::get_logger("ArduinoHardwareInterface"), "READ: Failed to receive 'S ' status line from Arduino after multiple attempts.");
        return hardware_interface::return_type::ERROR;
    }

    std::stringstream ss(response_line.substr(2)); // Skip "S "
    std::vector<int> current_arduino_steps(joint_names_in_arduino_order_.size());
    bool parse_ok = true;

    for (size_t i = 0; i < joint_names_in_arduino_order_.size(); ++i) {
        if (!(ss >> current_arduino_steps[i])) {
            RCLCPP_ERROR(rclcpp::get_logger("ArduinoHardwareInterface"), "READ: Failed to parse step value for Arduino joint index %zu from status: '%s'", i, response_line.c_str());
            parse_ok = false;
            break;
        }
    }
    if (!parse_ok) return hardware_interface::return_type::ERROR;

    double period_seconds = period.seconds();
    if (period_seconds <= 0.0) { // Avoid division by zero if period is invalid
        RCLCPP_WARN(rclcpp::get_logger("ArduinoHardwareInterface"), "READ: Period is zero or negative (%f s), cannot calculate velocity.", period_seconds);
        period_seconds = 1.0; // Avoid NaN, but velocity will be inaccurate
    }

    for (size_t i = 0; i < joint_names_in_arduino_order_.size(); ++i) {
        const std::string& urdf_joint_name = joint_names_in_arduino_order_[i];
        // Find the joint index
        size_t urdf_joint_idx = 0;
        for (size_t j = 0; j < info_.joints.size(); ++j) {
            if (info_.joints[j].name == urdf_joint_name) {
                urdf_joint_idx = j;
                break;
            }
        }

        prev_hw_states_positions_[urdf_joint_idx] = hw_states_positions_[urdf_joint_idx]; // Store previous position
        hw_states_positions_[urdf_joint_idx] = arduinoToRosUnits(urdf_joint_name, current_arduino_steps[i]);
        
        if (std::isnan(prev_hw_states_positions_[urdf_joint_idx])) { // First read after activation
             hw_states_velocities_[urdf_joint_idx] = 0.0;
        } else {
             hw_states_velocities_[urdf_joint_idx] = (hw_states_positions_[urdf_joint_idx] - prev_hw_states_positions_[urdf_joint_idx]) / period_seconds;
        }
        
        RCLCPP_DEBUG(rclcpp::get_logger("ArduinoHardwareInterface"), "READ Joint '%s' (Arduino Idx %zu, URDF Idx %zu): Steps=%d -> PosState=%f, VelState=%f",
            urdf_joint_name.c_str(), i, urdf_joint_idx, current_arduino_steps[i], hw_states_positions_[urdf_joint_idx], hw_states_velocities_[urdf_joint_idx]);
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArduinoHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    std::stringstream command_stream;
    command_stream << "M:"; 

    for (size_t i = 0; i < joint_names_in_arduino_order_.size(); ++i) {
        const std::string& urdf_joint_name = joint_names_in_arduino_order_[i];
        // Find the joint index
        size_t urdf_joint_idx = 0;
        for (size_t j = 0; j < info_.joints.size(); ++j) {
            if (info_.joints[j].name == urdf_joint_name) {
                urdf_joint_idx = j;
                break;
            }
        }

        if (std::isnan(hw_commands_positions_[urdf_joint_idx])) {
            RCLCPP_ERROR(rclcpp::get_logger("ArduinoHardwareInterface"), "WRITE: NaN command for joint '%s'. Skipping write cycle.", urdf_joint_name.c_str());
            return hardware_interface::return_type::ERROR;
        }
        int target_steps = rosToArduinoSteps(urdf_joint_name, hw_commands_positions_[urdf_joint_idx]);
        command_stream << target_steps;
        if (i < joint_names_in_arduino_order_.size() - 1) {
            command_stream << ",";
        }
        RCLCPP_DEBUG(rclcpp::get_logger("ArduinoHardwareInterface"), "WRITE Joint '%s' (Arduino Idx %zu, URDF Idx %zu): Command Pos=%f -> Steps=%d",
            urdf_joint_name.c_str(), i, urdf_joint_idx, hw_commands_positions_[urdf_joint_idx], target_steps);
    }
    command_stream << "\n"; 

    if (!sendSerialCommand(command_stream.str())) {
        RCLCPP_ERROR(rclcpp::get_logger("ArduinoHardwareInterface"), "WRITE: Failed to send move command '%s'", command_stream.str().c_str());
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

} // namespace arduino_hw_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  arduino_hw_interface::ArduinoHardwareInterface, hardware_interface::SystemInterface)
