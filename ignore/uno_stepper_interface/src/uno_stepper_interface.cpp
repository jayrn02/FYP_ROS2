#include "uno_stepper_interface/uno_stepper_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <sstream> // Required for std::stringstream
#include <iomanip> // Required for std::fixed, std::setprecision
#include <thread>  // Required for std::this_thread
#include <algorithm> // Required for std::remove

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp> // Include Boost.Asio

// --- Add includes for logging severity ---
#include "rclcpp/logging.hpp"
// ---

using namespace std::chrono_literals; // For ms suffix

namespace uno_stepper_interface
{

// --- on_init ---
CallbackReturn UnoStepperInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "on_init starting");

    // --- Configuration ---
    // Use cfg_ member directly
    cfg_.serial_port_name = info_.hardware_parameters["serial_port_name"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    // --- Read joint names ---
    cfg_.joint_names.reserve(info_.joints.size());
    for (const auto& joint : info_.joints) {
        cfg_.joint_names.push_back(joint.name);
        RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Registered joint: %s", joint.name.c_str());
    }
    // --- Read home steps ---
    std::string home_steps_str = info_.hardware_parameters["home_steps"];
    std::stringstream ss_home(home_steps_str);
    long step_val;
    while (ss_home >> step_val) {
        home_steps_.push_back(step_val);
    }
    if (home_steps_.size() != cfg_.joint_names.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Mismatch between number of joints (%ld) and home_steps (%ld)", cfg_.joint_names.size(), home_steps_.size());
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Home steps loaded: %s", home_steps_str.c_str());


    // --- Resize state and command vectors ---
    // Use correct member variable names
    hw_states_positions_.resize(cfg_.joint_names.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_velocities_.resize(cfg_.joint_names.size(), std::numeric_limits<double>::quiet_NaN()); // Assuming velocity is not directly controlled/read
    hw_commands_positions_.resize(cfg_.joint_names.size(), std::numeric_limits<double>::quiet_NaN());

    // --- Initialize Serial Port ---
    try {
        serial_port_ = std::make_unique<boost::asio::serial_port>(io_context_);
        RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Attempting to open serial port: %s at %d baud", cfg_.serial_port_name.c_str(), cfg_.baud_rate);
        serial_port_->open(cfg_.serial_port_name);
        serial_port_->set_option(boost::asio::serial_port_base::baud_rate(cfg_.baud_rate));
        serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
        serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Serial port opened successfully.");
    } catch (const boost::system::system_error& e) {
        RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Failed to open serial port %s: %s", cfg_.serial_port_name.c_str(), e.what());
        serial_port_.reset(); // Ensure it's null if open failed
        return CallbackReturn::ERROR;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Unexpected error during serial port initialization: %s", e.what());
        serial_port_.reset();
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "System Successfully configured!");
    return CallbackReturn::SUCCESS;
}

// --- export_state_interfaces ---
std::vector<hardware_interface::StateInterface> UnoStepperInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); ++i)
    {
        // Use correct member variable name
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
        // If velocity is tracked, add it here too (using correct name)
        // state_interfaces.emplace_back(hardware_interface::StateInterface(
        //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    }
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "State interfaces exported.");
    return state_interfaces;
}

// --- export_command_interfaces ---
std::vector<hardware_interface::CommandInterface> UnoStepperInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); ++i)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    }
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Command interfaces exported.");
    return command_interfaces;
}

// --- on_configure ---
CallbackReturn UnoStepperInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Configuring...");
    // Reset command and state vectors
    // Use correct member variable names
    for (double &pos : hw_states_positions_) { pos = std::numeric_limits<double>::quiet_NaN(); }
    for (double &vel : hw_states_velocities_) { vel = std::numeric_limits<double>::quiet_NaN(); }
    for (double &cmd : hw_commands_positions_) { cmd = std::numeric_limits<double>::quiet_NaN(); }

    // Check if serial port is still open (it should be if on_init succeeded)
    if (!serial_port_ || !serial_port_->is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Serial port not open during configure!");
        // Attempt to reopen? Or just fail? For now, fail.
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Configuration successful.");
    return CallbackReturn::SUCCESS;
}

// --- on_cleanup ---
CallbackReturn UnoStepperInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Cleaning up...");
    // Close serial port if open
    if (serial_port_ && serial_port_->is_open()) {
        RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Closing serial port.");
        try {
            serial_port_->close();
        } catch (const boost::system::system_error& e) {
            RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Error closing serial port: %s", e.what());
            // Continue cleanup even if close fails
        }
    }
    serial_port_.reset(); // Release the unique_ptr
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Cleanup successful.");
    return CallbackReturn::SUCCESS;
}

// --- on_shutdown ---
CallbackReturn UnoStepperInterface::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Shutting down...");
    // Perform cleanup, similar to on_cleanup
    return on_cleanup(previous_state);
}

// --- on_activate ---
CallbackReturn UnoStepperInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Activating...");
    if (!serial_port_ || !serial_port_->is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Serial port not open on activation!");
        return CallbackReturn::ERROR;
    }

    // Clear any stale data from the serial buffer
    read_asio_buffer_.consume(read_asio_buffer_.size()); // Clear Asio streambuf
    last_read_msg_.clear();
    RCLCPP_DEBUG(rclcpp::get_logger("UnoStepperInterface"), "Cleared serial read buffer.");

    // Wait for potential Arduino reset after serial connection established
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Waiting for potential Arduino reset (2 seconds)...");
    std::this_thread::sleep_for(2000ms);

    // --- Homing Sequence ---
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Sending Home command (H)...");
    if (!sendSerialCommand("H\n", "OK\n")) {
        RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Failed to home robot. Did not receive 'OK\\n' after 'H\\n'. Check Arduino logs.");
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Homing acknowledged by Arduino.");

    // --- Enable Steppers ---
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Sending Enable command (E1)...");
     if (!sendSerialCommand("E1\n", "OK\n")) {
         RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Failed to enable steppers. Did not receive 'OK\\n' after 'E1\\n'. Check Arduino logs.");
         return CallbackReturn::ERROR;
     }
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Steppers enabled.");

    // Reset commands to NaN to prevent immediate movement if controller starts with old commands
    for (double &cmd : hw_commands_positions_) { cmd = std::numeric_limits<double>::quiet_NaN(); }
    // Set initial state to home positions (assuming homing sets them to the configured home_steps_)
    // Use correct member variable name
    for (size_t i = 0; i < hw_states_positions_.size(); ++i) {
        hw_states_positions_[i] = static_cast<double>(home_steps_[i]); // Use configured home steps
    }

    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Activation successful. Initial state set to home positions.");
    return CallbackReturn::SUCCESS;
}

// --- on_deactivate ---
CallbackReturn UnoStepperInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Deactivating...");
    if (serial_port_ && serial_port_->is_open()) {
        RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Sending Disable command (E0)...");
        // Send best effort, don't wait for response as we are deactivating
        sendSerialCommand("E0\n", "", false);
    } else {
        RCLCPP_WARN(rclcpp::get_logger("UnoStepperInterface"), "Serial port closed or unavailable during deactivation.");
    }
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Deactivation finished.");
    return CallbackReturn::SUCCESS;
}

// --- on_error ---
// This function definition should now match the declaration in the header
CallbackReturn UnoStepperInterface::on_error(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Error occurred. Entering error state.");
    // Optionally try to close the serial port cleanly
    if (serial_port_ && serial_port_->is_open()) {
        RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Closing serial port due to error.");
        try {
            serial_port_->close();
        } catch (const boost::system::system_error& e) {
            RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Error closing serial port during error handling: %s", e.what());
        }
    }
    serial_port_.reset();
    return CallbackReturn::SUCCESS; // Indicate error state transition is handled
}


// --- read ---
hardware_interface::return_type UnoStepperInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!serial_port_ || !serial_port_->is_open()) {
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("UnoStepperInterface"), *rclcpp::Clock::make_shared(), 1000, "Serial port not open for reading.");
        return hardware_interface::return_type::ERROR;
    }

    RCLCPP_DEBUG(rclcpp::get_logger("UnoStepperInterface"), "Read function called.");

    // --- Process existing data in buffer first ---
    std::istream is(&read_asio_buffer_);
    std::string line;
    while (std::getline(is, line)) {
        line.erase(std::remove(line.begin(), line.end(), '\r'), line.end()); // Remove carriage returns
        last_read_msg_ = line;
        RCLCPP_DEBUG(rclcpp::get_logger("UnoStepperInterface"), "Processed line from buffer: '%s'", last_read_msg_.c_str());

        if (last_read_msg_.rfind("S ", 0) == 0) { // Check if it starts with "S "
            if (!parseStatusString(last_read_msg_.substr(2))) { // Pass substring after "S "
                RCLCPP_WARN(rclcpp::get_logger("UnoStepperInterface"), "Failed to parse status string: '%s'", last_read_msg_.c_str());
            } else {
                 RCLCPP_DEBUG(rclcpp::get_logger("UnoStepperInterface"), "Successfully parsed status string.");
            }
        }
        // --- Log Arduino Output ---
        else if (last_read_msg_.rfind("INFO:", 0) == 0 ||
                   last_read_msg_.rfind("DEBUG:", 0) == 0 ||
                   last_read_msg_.rfind("ERROR:", 0) == 0)
        {
            // Log Arduino's own log messages at INFO level in ROS
            RCLCPP_INFO(rclcpp::get_logger("ArduinoOutput"), "%s", last_read_msg_.c_str());
        }
        // --- End Log Arduino Output ---
         else if (!last_read_msg_.empty() && last_read_msg_ != "OK") { // Ignore empty lines and simple "OK" responses here
            // Log other unexpected messages
            RCLCPP_WARN(rclcpp::get_logger("UnoStepperInterface"), "Unexpected Arduino Msg in read loop: '%s'", last_read_msg_.c_str());
        }
    }
    // After processing, clear the stream state ONLY if EOF was reached (meaning we processed all full lines)
    if (is.eof()) {
        is.clear(); // Clear EOF state so the buffer can be used again
        RCLCPP_DEBUG(rclcpp::get_logger("UnoStepperInterface"), "Finished processing existing buffer.");
    } else {
         RCLCPP_DEBUG(rclcpp::get_logger("UnoStepperInterface"), "Partial line may remain in buffer.");
    }


    // --- Attempt to read more data ---
    // This read_until is BLOCKING. If the Arduino doesn't send a newline,
    // this will hang. A more robust implementation would use async reads or timeouts.
    RCLCPP_DEBUG(rclcpp::get_logger("UnoStepperInterface"), "Attempting blocking read_until '\\n'...");
    boost::system::error_code ec;
    size_t bytes_read = boost::asio::read_until(*serial_port_, read_asio_buffer_, '\n', ec);
    RCLCPP_DEBUG(rclcpp::get_logger("UnoStepperInterface"), "read_until finished. Bytes read: %ld, Error code: %s (%d)", bytes_read, ec.message().c_str(), ec.value());

    if (ec) {
        // Don't treat EOF as a critical error here, it just means the port might have closed or buffer ended.
        // Other errors (like device not configured) are more serious.
        if (ec != boost::asio::error::eof && ec != boost::asio::error::operation_aborted) {
            RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("UnoStepperInterface"), *rclcpp::Clock::make_shared(), 1000, "Error during serial read_until: %s", ec.message().c_str());
            // Consider returning ERROR only for specific, persistent errors
            // return hardware_interface::return_type::ERROR;
        } else if (ec == boost::asio::error::eof) {
             RCLCPP_WARN_THROTTLE(rclcpp::get_logger("UnoStepperInterface"), *rclcpp::Clock::make_shared(), 5000, "EOF encountered during serial read. Port may have closed.");
        }
    }

    // --- Process newly read data (if any) ---
    // This duplicates the processing logic from above. Consider refactoring into a helper function.
    std::istream is_new(&read_asio_buffer_);
    while (std::getline(is_new, line)) {
        line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
        last_read_msg_ = line;
        RCLCPP_DEBUG(rclcpp::get_logger("UnoStepperInterface"), "Processed newly read line: '%s'", last_read_msg_.c_str());

        if (last_read_msg_.rfind("S ", 0) == 0) {
            if (!parseStatusString(last_read_msg_.substr(2))) {
                RCLCPP_WARN(rclcpp::get_logger("UnoStepperInterface"), "Failed to parse newly read status string: '%s'", last_read_msg_.c_str());
            } else {
                 RCLCPP_DEBUG(rclcpp::get_logger("UnoStepperInterface"), "Successfully parsed newly read status string.");
            }
        } else if (last_read_msg_.rfind("INFO:", 0) == 0 || last_read_msg_.rfind("DEBUG:", 0) == 0 || last_read_msg_.rfind("ERROR:", 0) == 0) {
            RCLCPP_INFO(rclcpp::get_logger("ArduinoOutput"), "%s", last_read_msg_.c_str());
        } else if (!last_read_msg_.empty() && last_read_msg_ != "OK") {
            RCLCPP_WARN(rclcpp::get_logger("UnoStepperInterface"), "Unexpected Arduino Msg in read loop (new): '%s'", last_read_msg_.c_str());
        }
    }
     if (is_new.eof()) {
        is_new.clear();
    }

    return hardware_interface::return_type::OK;
}


// --- write ---
hardware_interface::return_type UnoStepperInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!serial_port_ || !serial_port_->is_open()) {
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("UnoStepperInterface"), *rclcpp::Clock::make_shared(), 1000, "Serial port not open for writing.");
        return hardware_interface::return_type::ERROR;
    }

    RCLCPP_DEBUG(rclcpp::get_logger("UnoStepperInterface"), "Write function called.");

    std::string command_str = "P"; // Position command prefix
    bool command_changed = false;

    for (size_t i = 0; i < hw_commands_positions_.size(); ++i) {
        long target_steps;
        // Check if command is NaN - if so, maybe send current position or home?
        // For now, let's assume NaN means "don't change" or use home position.
        // Using home position if NaN might be safer initially.
        if (std::isnan(hw_commands_positions_[i])) {
            // Option 1: Send home position (safer if controller starts with NaN)
             target_steps = home_steps_[i];
            // Option 2: Send current known position (might drift if read fails)
            // Use correct member variable name
            // target_steps = static_cast<long>(hw_states_positions_[i]); // Requires valid hw_states_positions_
            // Option 3: Do nothing / skip sending for this joint (needs command format adjustment)
        } else {
            // Convert command (radians/meters) to steps if necessary - ASSUMING commands are already in steps
            target_steps = static_cast<long>(hw_commands_positions_[i]);

            // Check if the command has actually changed from the last known state (optional optimization)
            // Note: hw_states_positions_ might not be perfectly up-to-date
            // Use correct member variable name
            // if (target_steps != static_cast<long>(hw_states_positions_[i])) {
            //     command_changed = true;
            // }
            command_changed = true; // Assume change for simplicity now
        }
        command_str += " " + std::to_string(target_steps);
    }
    command_str += "\n"; // Add newline terminator

    // Only send if at least one command was valid and potentially changed
    if(command_changed) {
        RCLCPP_DEBUG(rclcpp::get_logger("UnoStepperInterface"), "Sending Command: %s", command_str.c_str());
        try {
            boost::system::error_code ec;
            boost::asio::write(*serial_port_, boost::asio::buffer(command_str), ec);
            if (ec) {
                 RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Error sending command '%s': %s", command_str.substr(0, command_str.find('\n')).c_str(), ec.message().c_str());
                 return hardware_interface::return_type::ERROR;
            } else {
                 RCLCPP_DEBUG(rclcpp::get_logger("UnoStepperInterface"), "Command sent successfully.");
            }
        } catch (const boost::system::system_error& bse) {
             RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Boost system_error sending command: %s", bse.what());
             return hardware_interface::return_type::ERROR;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Exception sending command: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
    } else {
         RCLCPP_DEBUG(rclcpp::get_logger("UnoStepperInterface"), "Write function: No command change detected, not sending.");
    }

    return hardware_interface::return_type::OK;
}

// --- sendSerialCommand ---
bool UnoStepperInterface::sendSerialCommand(const std::string& command, const std::string& wait_for, bool log_error /*= true*/)
{
    if (!serial_port_ || !serial_port_->is_open()) {
        if (log_error) RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Serial port not open for sendSerialCommand.");
        return false;
    }

    RCLCPP_DEBUG(rclcpp::get_logger("UnoStepperInterface"), "Sending command: '%s', waiting for: '%s'", command.substr(0, command.find('\n')).c_str(), wait_for.empty() ? "nothing" : wait_for.substr(0, wait_for.find('\n')).c_str());

    try {
        // Write command
        boost::system::error_code ec_write;
        boost::asio::write(*serial_port_, boost::asio::buffer(command), ec_write);
        if (ec_write) {
            if (log_error) RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Error sending command '%s': %s", command.substr(0, command.find('\n')).c_str(), ec_write.message().c_str());
            return false;
        }
        RCLCPP_DEBUG(rclcpp::get_logger("UnoStepperInterface"), "Command '%s' written to serial.", command.substr(0, command.find('\n')).c_str());

        // If no response expected, return success
        if (wait_for.empty()) {
            RCLCPP_DEBUG(rclcpp::get_logger("UnoStepperInterface"), "No response needed for command '%s'.", command.substr(0, command.find('\n')).c_str());
            return true;
        }

        // Read response with timeout
        // NOTE: readLineWithTimeout currently BLOCKS indefinitely if no newline arrives.
        // The timeout parameter is not actually used in the current implementation.
        RCLCPP_DEBUG(rclcpp::get_logger("UnoStepperInterface"), "Waiting for response '%s'...", wait_for.substr(0, wait_for.find('\n')).c_str());
        std::string line;
        // Use cfg_ member directly
        if (readLineWithTimeout(line, std::chrono::milliseconds(cfg_.timeout_ms))) { // Pass configured timeout
             RCLCPP_DEBUG(rclcpp::get_logger("UnoStepperInterface"), "Received response line: '%s'", line.c_str());
            if (line.find(wait_for) != std::string::npos) {
                 RCLCPP_DEBUG(rclcpp::get_logger("UnoStepperInterface"), "Expected response '%s' received.", wait_for.substr(0, wait_for.find('\n')).c_str());
                return true; // Found expected response
            } else {
                 // Log unexpected response
                 if (log_error) RCLCPP_WARN(rclcpp::get_logger("UnoStepperInterface"), "Received '%s' instead of expected '%s' after sending '%s'", line.c_str(), wait_for.substr(0, wait_for.find('\n')).c_str(), command.substr(0, command.find('\n')).c_str());
                 // Check if the unexpected line is an Arduino log message and log it appropriately
                 if (line.rfind("INFO:", 0) == 0 || line.rfind("DEBUG:", 0) == 0 || line.rfind("ERROR:", 0) == 0) {
                     RCLCPP_INFO(rclcpp::get_logger("ArduinoOutput"), "%s", line.c_str()); // Log it via ArduinoOutput logger
                 }
                 // Even if it was an Arduino log, we didn't get the expected response
                 return false;
            }
        } else {
             // Timeout occurred (or readLineWithTimeout failed for other reasons)
             // Since readLineWithTimeout doesn't actually timeout, this branch might indicate a different read error.
             if (log_error) RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "readLineWithTimeout failed or timed out waiting for response '%s' after sending '%s'", wait_for.substr(0, wait_for.find('\n')).c_str(), command.substr(0, command.find('\n')).c_str());
            return false;
        }
    } catch (const boost::system::system_error& bse) {
         if (log_error) RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Boost system_error in sendSerialCommand: %s", bse.what());
         return false;
    } catch (const std::exception& e) {
        if (log_error) RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Exception in sendSerialCommand: %s", e.what());
        return false;
    }
    // Should not be reached
    return false;
}


// --- readLineWithTimeout --- Basic non-blocking check, real timeout would need async I/O
// WARNING: This implementation is effectively BLOCKING if no newline arrives.
// The 'timeout' parameter is currently ignored by boost::asio::read_until.
// A true non-blocking read with timeout is more complex with raw Boost.Asio.
bool UnoStepperInterface::readLineWithTimeout(std::string& line, [[maybe_unused]] std::chrono::milliseconds timeout) {
     if (!serial_port_ || !serial_port_->is_open()) {
         RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("UnoStepperInterface"), *rclcpp::Clock::make_shared(), 1000, "readLineWithTimeout: Serial port not open.");
         return false;
     }
    try {
         boost::system::error_code ec;
         line.clear();

         // This effectively blocks until a newline or error
         RCLCPP_DEBUG(rclcpp::get_logger("UnoStepperInterface"), "Calling blocking boost::asio::read_until for '\\n'...");
         boost::asio::read_until(*serial_port_, read_asio_buffer_, '\n', ec);
         RCLCPP_DEBUG(rclcpp::get_logger("UnoStepperInterface"), "boost::asio::read_until finished. Error code: %s (%d)", ec.message().c_str(), ec.value());

         if (ec) {
             // Handle errors like EOF or port closed
             if (ec != boost::asio::error::eof && ec != boost::asio::error::operation_aborted) {
                 RCLCPP_WARN(rclcpp::get_logger("UnoStepperInterface"), "readLineWithTimeout read_until error: %s", ec.message().c_str());
             } else if (ec == boost::asio::error::eof) {
                 RCLCPP_WARN(rclcpp::get_logger("UnoStepperInterface"), "readLineWithTimeout read_until EOF.");
             }
             // Regardless of the specific error, we didn't successfully read a line up to '\n'
             return false;
         }

         // If read_until succeeded, extract the line from the buffer
         std::istream is(&read_asio_buffer_);
         if (std::getline(is, line)) {
             line.erase(std::remove(line.begin(), line.end(), '\r'), line.end()); // Remove CR
             RCLCPP_DEBUG(rclcpp::get_logger("UnoStepperInterface"), "Successfully read line: '%s'", line.c_str());
             return true;
         } else {
             // This case should be unlikely if read_until succeeded without error,
             // but handle it just in case.
             RCLCPP_WARN(rclcpp::get_logger("UnoStepperInterface"), "readLineWithTimeout: std::getline failed after successful read_until.");
             return false;
         }

    } catch (const boost::system::system_error& bse) {
        RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Boost system_error in readLineWithTimeout: %s", bse.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Exception in readLineWithTimeout: %s", e.what());
        return false;
    }
}


// --- parseStatusString ---
bool UnoStepperInterface::parseStatusString(const std::string& status_str_args) {
    RCLCPP_DEBUG(rclcpp::get_logger("UnoStepperInterface"), "Parsing status string args: '%s'", status_str_args.c_str());
    std::stringstream ss(status_str_args);
    long step_val;
    std::vector<long> current_steps;
    while (ss >> step_val) {
        current_steps.push_back(step_val);
    }

    // Use correct member variable name
    if (current_steps.size() == hw_states_positions_.size()) {
        for (size_t i = 0; i < current_steps.size(); ++i) {
            // Assuming status string gives steps directly
            // Use correct member variable name
            hw_states_positions_[i] = static_cast<double>(current_steps[i]);
            // Velocity is not provided by this status string format
            // Use correct member variable name
            hw_states_velocities_[i] = std::numeric_limits<double>::quiet_NaN();
        }
         RCLCPP_DEBUG(rclcpp::get_logger("UnoStepperInterface"), "Parsed %ld joint steps.", current_steps.size());
        return true;
    } else {
        // Use correct member variable name
        RCLCPP_WARN(rclcpp::get_logger("UnoStepperInterface"), "Parsed %ld values, expected %ld for joint states from status string '%s'", current_steps.size(), hw_states_positions_.size(), status_str_args.c_str());
        return false; // Added missing return
    }
    // return false; // Should not be reached if logic is correct, but added for safety
}


} // namespace uno_stepper_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(uno_stepper_interface::UnoStepperInterface, hardware_interface::SystemInterface)