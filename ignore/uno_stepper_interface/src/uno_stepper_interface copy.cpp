#include "uno_stepper_interface/uno_stepper_interface.hpp"

#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <chrono>
#include <thread>
#include <sstream>
#include <algorithm>
#include <iostream>

#include "rclcpp/logging.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// Boost includes are in the header

using namespace std::chrono_literals;

namespace uno_stepper_interface
{

// --- on_init (Correct) ---
CallbackReturn UnoStepperInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) { return CallbackReturn::ERROR; }
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Initializing UnoStepperInterface (Boost.Asio)...");
    // Read Parameters...
    serial_port_name_ = info_.hardware_parameters["serial_port"];
    baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
    timeout_ms_ = std::chrono::milliseconds(std::stoi(info_.hardware_parameters["timeout_ms"]));
    gear_ratio_ = std::stod(info_.hardware_parameters["gear_ratio"]);
    steps_per_rev_ = std::stod(info_.hardware_parameters["steps_per_rev"]);
    microstepping_ = std::stod(info_.hardware_parameters["microstepping"]);
    gripper_steps_closed_ = std::stod(info_.hardware_parameters["gripper_steps_closed"]);
    gripper_steps_open_ = std::stod(info_.hardware_parameters["gripper_steps_open"]);
    gripper_meters_closed_ = std::stod(info_.hardware_parameters["gripper_meters_closed"]);
    gripper_meters_open_ = std::stod(info_.hardware_parameters["gripper_meters_open"]);
    // Read Joint Info...
    if (info_.joints.empty()) { RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "No joints defined!"); return CallbackReturn::ERROR; }
    joint_names_.resize(info_.joints.size());
    hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    home_steps_.resize(info_.joints.size());
    if (info_.hardware_parameters.find("joint_names") == info_.hardware_parameters.end()) { RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Param 'joint_names' not found!"); return CallbackReturn::ERROR; }
    std::stringstream ss_joints(info_.hardware_parameters["joint_names"]);
    std::string joint_name;
    int joint_idx = 0;
    while (std::getline(ss_joints, joint_name, ',')) {
         joint_name.erase(0, joint_name.find_first_not_of(" \t\n\r\f\v"));
         joint_name.erase(joint_name.find_last_not_of(" \t\n\r\f\v") + 1);
         if (static_cast<size_t>(joint_idx) >= info_.joints.size()) { RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "More joints in 'joint_names' param than URDF!"); return CallbackReturn::ERROR; }
         joint_names_[joint_idx] = joint_name;
         bool found_in_urdf = false;
         for(const auto& urdf_joint : info_.joints) { if (urdf_joint.name == joint_name) { found_in_urdf = true; if (urdf_joint.state_interfaces.empty() || urdf_joint.command_interfaces.empty()){ RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Joint '%s' missing interfaces!", joint_name.c_str()); return CallbackReturn::ERROR; } break; } }
         if (!found_in_urdf) { RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Joint '%s' from param not in URDF!", joint_name.c_str()); return CallbackReturn::ERROR; }
         std::string home_param_name;
         if (static_cast<size_t>(joint_idx) == info_.joints.size() - 1) { home_steps_[joint_idx] = static_cast<long>(gripper_steps_closed_);
         } else { if (joint_idx == 0) home_param_name = "home_steps_base"; else if (joint_idx == 1) home_param_name = "home_steps_shoulder"; else if (joint_idx == 2) home_param_name = "home_steps_elbow"; if (info_.hardware_parameters.find(home_param_name) == info_.hardware_parameters.end()) { RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Param '%s' not found!", home_param_name.c_str()); return CallbackReturn::ERROR; } home_steps_[joint_idx] = std::stol(info_.hardware_parameters.at(home_param_name)); }
         // RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "  Joint %d: %s, Home Steps: %ld", joint_idx, joint_names_[joint_idx].c_str(), home_steps_[joint_idx]);
         joint_idx++;
     }
     if (static_cast<size_t>(joint_idx) != info_.joints.size()) { RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Mismatch URDF joints vs param joints"); return CallbackReturn::ERROR; }
    // Calculate Conversion Factors...
    double steps_per_rev_total = steps_per_rev_ * microstepping_;
    rad_to_step_factor_ = gear_ratio_ * steps_per_rev_total / (2.0 * M_PI);
    step_to_rad_factor_ = 1.0 / rad_to_step_factor_;
    double gripper_steps_range = gripper_steps_open_ - gripper_steps_closed_;
    double gripper_meters_range = gripper_meters_open_ - gripper_meters_closed_;
    if (std::abs(gripper_steps_range) < 1e-6 || std::abs(gripper_meters_range) < 1e-9) { RCLCPP_WARN(rclcpp::get_logger("UnoStepperInterface"), "Gripper range zero."); gripper_meters_to_steps_factor_ = 1.0; gripper_steps_to_meters_factor_ = 0.0; }
    else { gripper_meters_to_steps_factor_ = gripper_steps_range / gripper_meters_range; gripper_steps_to_meters_factor_ = gripper_meters_range / gripper_steps_range; }
    // Logging...
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Parameters loaded:");
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "  Port: %s, Baud: %d, Timeout: %ld ms", serial_port_name_.c_str(), baud_rate_, timeout_ms_.count());
    // ... etc
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Initialization successful.");
    return CallbackReturn::SUCCESS;
}

// --- on_configure (Correct) ---
CallbackReturn UnoStepperInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Configuring...");
    for (size_t i = 0; i < hw_states_positions_.size(); ++i) { hw_states_positions_[i]=NAN; hw_states_velocities_[i]=NAN; hw_commands_positions_[i]=NAN;} // Reset states/commands
    try {
        serial_port_ = std::make_unique<boost::asio::serial_port>(io_context_, serial_port_name_);
        serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
        serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
        serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    } catch (const boost::system::system_error &e) { RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Failed to open/config serial port '%s': %s (%d)", serial_port_name_.c_str(), e.what(), e.code().value()); serial_port_.reset(); return CallbackReturn::ERROR;
    } catch (const std::exception &e) { RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Exception opening serial port '%s': %s", serial_port_name_.c_str(), e.what()); serial_port_.reset(); return CallbackReturn::ERROR; }
    if (!serial_port_ || !serial_port_->is_open()) { RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Serial port null or not open after config."); return CallbackReturn::ERROR; }
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Serial port opened successfully: %s", serial_port_name_.c_str());
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Configuration successful.");
    return CallbackReturn::SUCCESS;
}

// --- export_state_interfaces (Correct) ---
std::vector<hardware_interface::StateInterface> UnoStepperInterface::export_state_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Exporting state interfaces...");
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < joint_names_.size(); ++i) // Use joint_names_.size()
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    }
    return state_interfaces; // Return the vector
}

// --- export_command_interfaces (Correct) ---
std::vector<hardware_interface::CommandInterface> UnoStepperInterface::export_command_interfaces()
{
     RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Exporting command interfaces...");
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < joint_names_.size(); ++i) // Use joint_names_.size()
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    }
    return command_interfaces; // Return the vector
}

// --- on_activate (Correct) ---
CallbackReturn UnoStepperInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Activating...");
    if (!serial_port_ || !serial_port_->is_open()) { RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Serial port not open on activation!"); return CallbackReturn::ERROR; }
    read_asio_buffer_.consume(read_asio_buffer_.size()); // Clear Asio streambuf
    last_read_msg_.clear();
    std::this_thread::sleep_for(2000ms);
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Sending Home command (H)...");
    if (!sendSerialCommand("H\n", "OK\n")) { RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Failed to home robot. No 'OK'."); return CallbackReturn::ERROR; }
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Homing acknowledged by Arduino.");
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Sending Enable command (E1)...");
     if (!sendSerialCommand("E1\n", "OK\n")) { RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Failed to enable steppers. No 'OK'."); return CallbackReturn::ERROR; }
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Steppers enabled.");
    for (size_t i = 0; i < hw_states_positions_.size(); ++i) {
         if (i == hw_states_positions_.size() - 1) { hw_states_positions_[i] = gripper_meters_closed_; }
         else { hw_states_positions_[i] = 0.0; }
        hw_states_velocities_[i] = 0.0;
        hw_commands_positions_[i] = hw_states_positions_[i];
    }
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Activation successful.");
    return CallbackReturn::SUCCESS;
}

// --- on_deactivate (Correct) ---
CallbackReturn UnoStepperInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Deactivating...");
    if (serial_port_ && serial_port_->is_open()) {
        RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Sending Disable command (E0)...");
        sendSerialCommand("E0\n", "", false); // Send best effort
    } else { RCLCPP_WARN(rclcpp::get_logger("UnoStepperInterface"), "Serial port closed or unavailable during deactivation."); }
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Deactivation successful.");
    return CallbackReturn::SUCCESS; // Added return
}

// --- on_cleanup (Correct) ---
CallbackReturn UnoStepperInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Cleaning up...");
    if (serial_port_ && serial_port_->is_open()) {
        try { serial_port_->close(); RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Serial port closed."); }
        catch (const std::exception &e) { RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Failed to close serial port: %s", e.what()); }
    }
    serial_port_.reset(); // Release unique_ptr
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Cleanup successful.");
    return CallbackReturn::SUCCESS; // Added return
}

// --- on_shutdown (Correct) ---
CallbackReturn UnoStepperInterface::on_shutdown(const rclcpp_lifecycle::State & [[maybe_unused]] previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Shutting down...");
    // Call deactivate and cleanup explicitly if needed, though lifecycle manager might do it.
    // on_deactivate(previous_state); // Might be redundant
    on_cleanup(previous_state); // Ensure port is closed
    RCLCPP_INFO(rclcpp::get_logger("UnoStepperInterface"), "Shutdown successful.");
    return CallbackReturn::SUCCESS;
}


// --- read (Correct) ---
hardware_interface::return_type UnoStepperInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!serial_port_ || !serial_port_->is_open()) { RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("UnoStepperInterface"), *rclcpp::Clock::make_shared(), 1000, "Serial port not open for reading."); return hardware_interface::return_type::ERROR; }
    try {
        boost::system::error_code ec;
        // Process any existing complete lines in the buffer first
        std::istream is_existing(&read_asio_buffer_);
        std::string line_existing;
        while (std::getline(is_existing, line_existing)) {
            line_existing.erase(std::remove(line_existing.begin(), line_existing.end(), '\r'), line_existing.end());
            last_read_msg_ = line_existing;
            if (last_read_msg_.rfind("S ", 0) == 0) { if (!parseStatusString(last_read_msg_.substr(2))) { /* Warn */ } }
            // ... handle other message types ...
        }
        // Attempt to read more data until a newline (blocking)
        boost::asio::read_until(*serial_port_, read_asio_buffer_, '\n', ec);
        if (ec && ec != boost::asio::error::eof && ec != boost::asio::error::operation_aborted) { RCLCPP_WARN_THROTTLE(rclcpp::get_logger("UnoStepperInterface"), *rclcpp::Clock::make_shared(), 1000, "Error during serial read_until: %s", ec.message().c_str()); }
        // Process any newly read complete lines
         std::istream is_new(&read_asio_buffer_);
         std::string line_new;
         while (std::getline(is_new, line_new)) {
            line_new.erase(std::remove(line_new.begin(), line_new.end(), '\r'), line_new.end());
            last_read_msg_ = line_new;
            if (last_read_msg_.rfind("S ", 0) == 0) { if (!parseStatusString(last_read_msg_.substr(2))) { /* Warn */ } }
            // ... handle other message types ...
        }
    } catch (const std::exception &e) { RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("UnoStepperInterface"), *rclcpp::Clock::make_shared(), 1000,"Exception during serial read: %s", e.what()); return hardware_interface::return_type::ERROR; }
    std::fill(hw_states_velocities_.begin(), hw_states_velocities_.end(), 0.0);
    return hardware_interface::return_type::OK;
}

// --- write (Correct) ---
hardware_interface::return_type UnoStepperInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!serial_port_ || !serial_port_->is_open()) { RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("UnoStepperInterface"), *rclcpp::Clock::make_shared(), 1000, "Serial port not open for writing."); return hardware_interface::return_type::ERROR; }
    std::string command_str = "P";
    bool command_changed = false;
    for (size_t i = 0; i < hw_commands_positions_.size(); ++i) {
        long target_steps;
        if (std::isnan(hw_commands_positions_[i])) { target_steps = home_steps_[i]; }
        else {
            command_changed = true;
            if (i == hw_commands_positions_.size() - 1) { // Gripper
                target_steps = static_cast<long>(round((hw_commands_positions_[i] - gripper_meters_closed_) * gripper_meters_to_steps_factor_ + gripper_steps_closed_));
                target_steps = std::max((long)gripper_steps_closed_, std::min((long)gripper_steps_open_, target_steps));
            } else { // Arm joints
                target_steps = static_cast<long>(round(hw_commands_positions_[i] * rad_to_step_factor_ + home_steps_[i]));
            }
        }
        command_str += " " + std::to_string(target_steps);
    }
    command_str += "\n";
    if(command_changed) {
        try {
            boost::system::error_code ec;
            boost::asio::write(*serial_port_, boost::asio::buffer(command_str), ec);
            if (ec) { RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Error sending command '%s': %s", command_str.substr(0, command_str.find('\n')).c_str(), ec.message().c_str()); return hardware_interface::return_type::ERROR; }
        } catch (const std::exception &e) { RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Exception sending command: %s", e.what()); return hardware_interface::return_type::ERROR; }
    }
    return hardware_interface::return_type::OK; // Added return
}


// --- sendSerialCommand (Correct) ---
bool UnoStepperInterface::sendSerialCommand(const std::string& command, const std::string& wait_for, bool log_error)
{
    if (!serial_port_ || !serial_port_->is_open()) { if (log_error) RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Serial port not open for send."); return false; }
    try {
        boost::system::error_code ec_write;
        boost::asio::write(*serial_port_, boost::asio::buffer(command), ec_write);
        if (ec_write) { if (log_error) RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Error sending command '%s': %s", command.substr(0, command.find('\n')).c_str(), ec_write.message().c_str()); return false; }
        if (wait_for.empty()) { return true; }
        std::string line;
        if (readLineWithTimeout(line, timeout_ms_)) {
            if (line.find(wait_for) != std::string::npos) { return true; }
            else { if (log_error) RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Received '%s' instead of expected '%s' after sending '%s'", line.c_str(), wait_for.c_str(), command.substr(0, command.find('\n')).c_str()); return false; }
        } else { if (log_error) RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Timeout waiting for response '%s' after sending '%s'", wait_for.c_str(), command.substr(0, command.find('\n')).c_str()); return false; }
    } catch (const std::exception &e) { if (log_error) RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Exception in sendSerialCommand: %s", e.what()); return false; }
    // Added default return false in case of unexpected flow
    return false;
}

// --- readLineWithTimeout (Correct - basic implementation) ---
bool UnoStepperInterface::readLineWithTimeout(std::string& line, [[maybe_unused]] std::chrono::milliseconds timeout) {
     if (!serial_port_ || !serial_port_->is_open()) return false;
    try {
         boost::system::error_code ec;
         line.clear();
         // This blocks indefinitely if no newline arrives. Timeout isn't handled.
         boost::asio::read_until(*serial_port_, read_asio_buffer_, '\n', ec);
         if (ec && ec != boost::asio::error::eof) { RCLCPP_WARN(rclcpp::get_logger("UnoStepperInterface"), "readLineWithTimeout error: %s", ec.message().c_str()); return false; }
         std::istream is(&read_asio_buffer_);
         if (std::getline(is, line)) {
             line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
             return true;
         } else { return false; }
    } catch (const std::exception& e){ RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Exception in readLineWithTimeout: %s", e.what()); return false; }
}

// --- parseStatusString (Correct) ---
bool UnoStepperInterface::parseStatusString(const std::string& status_str_args) {
    std::stringstream ss(status_str_args);
    std::vector<long> current_steps(hw_states_positions_.size());
    if (current_steps.size() != 4) { RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Internal error: Expected 4 states, got %ld", current_steps.size()); return false; }
    bool success = true;
    for (size_t i = 0; i < current_steps.size(); ++i) { if (!(ss >> current_steps[i])) { success = false; break; } }
    std::string remaining; ss >> remaining;
    if (!success || !remaining.empty()) { RCLCPP_WARN(rclcpp::get_logger("UnoStepperInterface"), "Status string format error or extra data: '%s'", status_str_args.c_str()); return false; }
    try {
        for (size_t i = 0; i < hw_states_positions_.size(); ++i) {
            if (i == hw_states_positions_.size() - 1) { hw_states_positions_[i] = (static_cast<double>(current_steps[i]) - gripper_steps_closed_) * gripper_steps_to_meters_factor_ + gripper_meters_closed_; }
            else { hw_states_positions_[i] = (static_cast<double>(current_steps[i]) - home_steps_[i]) * step_to_rad_factor_; } }
        std::fill(hw_states_velocities_.begin(), hw_states_velocities_.end(), 0.0);
    } catch (const std::exception& e) { RCLCPP_ERROR(rclcpp::get_logger("UnoStepperInterface"), "Exception during state conversion: %s", e.what()); return false; }
    return true;
}

} // namespace uno_stepper_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    uno_stepper_interface::UnoStepperInterface, hardware_interface::SystemInterface)