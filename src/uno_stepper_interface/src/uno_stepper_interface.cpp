#include "uno_stepper_interface/uno_stepper_interface.hpp"

#include <string>
#include <vector>
#include <cmath> // For M_PI, round
#include <chrono>
#include <stdexcept> // For exceptions
#include <sstream>   // For string streams
#include <iomanip>   // For setprecision
#include <algorithm> // For std::max, std::min

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <boost/asio/read_until.hpp>
#include <boost/asio/write.hpp>
#include <boost/asio/read.hpp> // Include for sync read
#include <boost/system/error_code.hpp>


namespace uno_stepper_interface
{

// --- Constructor ---
UnoStepperInterface::UnoStepperInterface()
    : logger_(rclcpp::get_logger("UnoStepperInterface")), // Initialize logger first
      io_context_(), // Initialize io_context
      serial_port_(io_context_) // Initialize serial port using io_context
{
    RCLCPP_INFO(logger_, "UnoStepperInterface constructor called.");
}

// --- Lifecycle Methods ---

CallbackReturn UnoStepperInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    RCLCPP_INFO(logger_, "on_init starting...");

    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        RCLCPP_ERROR(logger_, "Base SystemInterface::on_init failed.");
        return CallbackReturn::ERROR;
    }

    // --- Parameter Loading ---
    RCLCPP_INFO(logger_, "Loading hardware parameters...");
    try {
        // Helper lambda to check and get hardware parameter
        auto get_hw_param = [&](const std::string& param_name) -> std::string {
            if (info_.hardware_parameters.count(param_name)) {
                return info_.hardware_parameters.at(param_name);
            } else {
                std::string error_msg = "Required hardware parameter '" + param_name + "' not found in hardware_config.yaml.";
                RCLCPP_ERROR(logger_, "%s", error_msg.c_str());
                throw std::runtime_error(error_msg); // Throw specific error
            }
        };

        // Load hardware parameters with checks
        serial_port_name_ = get_hw_param("serial_port");
        RCLCPP_INFO(logger_, "  Serial Port: %s", serial_port_name_.c_str());

        baud_rate_ = std::stoi(get_hw_param("baud_rate"));
        RCLCPP_INFO(logger_, "  Baud Rate: %d", baud_rate_);

        serial_timeout_ms_ = std::stoi(get_hw_param("timeout_ms"));
        RCLCPP_INFO(logger_, "  Timeout (ms): %d", serial_timeout_ms_);

        steps_per_rev_ = std::stoi(get_hw_param("steps_per_revolution"));
        RCLCPP_INFO(logger_, "  Steps/Rev: %d", steps_per_rev_);

        microstepping_ = std::stoi(get_hw_param("microstepping"));
        RCLCPP_INFO(logger_, "  Microstepping: %d", microstepping_);

        gripper_range_meters_ = std::stod(get_hw_param("gripper_range_meters"));
        RCLCPP_INFO(logger_, "  Gripper Range (m): %.3f", gripper_range_meters_);

        gripper_range_steps_ = std::stol(get_hw_param("gripper_range_steps")); // std::stol handles conversion errors
        RCLCPP_INFO(logger_, "  Gripper Range (steps): %ld", gripper_range_steps_);


        // --- Joint Parameter Loading ---
        num_joints_ = info_.joints.size();
        RCLCPP_INFO(logger_, "Number of joints found: %zu", num_joints_);
        if (num_joints_ == 0) {
             RCLCPP_ERROR(logger_, "No joints defined in URDF/ros2_control tag.");
             return CallbackReturn::ERROR;
        }

        joint_names_.resize(num_joints_);
        gear_ratios_.resize(num_joints_);
        home_offsets_steps_.resize(num_joints_);
        hw_commands_positions_.resize(num_joints_, 0.0);
        hw_states_positions_.resize(num_joints_, 0.0);
        hw_states_velocities_.resize(num_joints_, 0.0);

        // Load parameters from info_.joints (defined in URDF <joint> tags)
        for (size_t i = 0; i < num_joints_; ++i) {
            joint_names_[i] = info_.joints[i].name;
            RCLCPP_INFO(logger_, "Processing joint: %s", joint_names_[i].c_str());

            // Helper lambda to check and get joint parameter
            auto get_joint_param = [&](const std::string& param_name) -> std::string {
                 if (info_.joints[i].parameters.count(param_name)) {
                     return info_.joints[i].parameters.at(param_name);
                 } else {
                     std::string error_msg = "Required joint parameter '" + param_name + "' not found for joint '" + joint_names_[i] + "' in URDF.";
                     RCLCPP_ERROR(logger_, "%s", error_msg.c_str());
                     throw std::runtime_error(error_msg); // Throw specific error
                 }
            };

            // Load gear ratio
            gear_ratios_[i] = std::stod(get_joint_param("gear_ratio"));
            RCLCPP_INFO(logger_, "  Gear Ratio for %s: %.2f", joint_names_[i].c_str(), gear_ratios_[i]);

            // Load home offset
            home_offsets_steps_[i] = std::stol(get_joint_param("home_offset_steps")); // std::stol handles conversion errors
            RCLCPP_INFO(logger_, "  Home Offset (steps) for %s: %ld", joint_names_[i].c_str(), home_offsets_steps_[i]);
        }

    } catch (const std::invalid_argument& ia) {
        // Catch errors from std::stoi/stod/stol (e.g., trying to convert "???" or non-numeric values)
        RCLCPP_ERROR(logger_, "Invalid parameter value format: %s. Check hardware_config.yaml or URDF joint parameters.", ia.what());
        return CallbackReturn::ERROR;
    } catch (const std::runtime_error& rte) {
        // Catch errors explicitly thrown by our get_hw_param/get_joint_param helpers
        RCLCPP_ERROR(logger_, "Parameter loading failed: %s", rte.what());
        return CallbackReturn::ERROR;
    } catch (const std::exception& e) {
        // Catch any other standard exceptions during parameter loading
        RCLCPP_ERROR(logger_, "Error during parameter loading: %s", e.what());
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger_, "Hardware parameters loaded successfully.");
    RCLCPP_INFO(logger_, "on_init finished successfully.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn UnoStepperInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger_, "on_configure called.");
    // Reset states before activation
    for (size_t i = 0; i < num_joints_; ++i) {
        // Consider loading initial positions from a file or parameter here if needed
        // hw_states_positions_[i] = initial_positions_[i];
        hw_states_positions_[i] = 0.0; // Default to 0
        hw_states_velocities_[i] = 0.0;
        hw_commands_positions_[i] = hw_states_positions_[i]; // Start command at current state
    }
    is_homed_ = false;
    is_enabled_ = false;
    is_serial_open_ = false; // Ensure flag is false before trying to open
    RCLCPP_INFO(logger_, "on_configure finished successfully.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn UnoStepperInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger_, "on_cleanup called.");
    close_serial_port(); // Ensure port is closed
    RCLCPP_INFO(logger_, "on_cleanup finished successfully.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn UnoStepperInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger_, "on_activate starting...");

    // --- Open Serial Port ---
    if (!open_serial_port()) {
        RCLCPP_ERROR(logger_, "Failed to open serial port during activation.");
        return CallbackReturn::ERROR;
    }

    // --- Homing Sequence ---
    RCLCPP_INFO(logger_, "Sending Homing command (H)...");
    if (!send_command("H\n")) {
        RCLCPP_ERROR(logger_, "Failed to send Homing command.");
        close_serial_port();
        return CallbackReturn::ERROR;
    }
    if (!wait_for_ok(std::chrono::milliseconds(15000))) { // Increased timeout for homing
        RCLCPP_ERROR(logger_, "Timeout or error waiting for OK after Homing command.");
        close_serial_port();
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger_, "Homing OK received.");
    is_homed_ = true;

    // --- Enable Motors ---
    RCLCPP_INFO(logger_, "Sending Enable command (E1)...");
    if (!send_command("E1\n")) {
         RCLCPP_ERROR(logger_, "Failed to send Enable command.");
        close_serial_port();
        return CallbackReturn::ERROR;
    }
     if (!wait_for_ok(std::chrono::milliseconds(serial_timeout_ms_))) {
        RCLCPP_ERROR(logger_, "Timeout or error waiting for OK after Enable command.");
        close_serial_port();
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger_, "Enable OK received.");
    is_enabled_ = true;

    // --- Request Initial Status ---
    RCLCPP_INFO(logger_, "Requesting initial status (S)...");
     if (!send_command("S\n")) {
         RCLCPP_ERROR(logger_, "Failed to send initial Status request command.");
        close_serial_port();
        return CallbackReturn::ERROR;
    }
    // Read initial status in the first read() call.
    // We could optionally add a loop here to wait for the first 'S' message.

    // Set commands to current states initially (states are still 0 until first read)
    for (size_t i = 0; i < num_joints_; ++i) {
         hw_commands_positions_[i] = hw_states_positions_[i];
    }

    RCLCPP_INFO(logger_, "on_activate finished successfully.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn UnoStepperInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger_, "on_deactivate starting...");
    if (is_serial_open_) {
        RCLCPP_INFO(logger_, "Sending Disable command (E0)...");
        if (!send_command("E0\n")) {
             RCLCPP_WARN(logger_, "Failed to send Disable command (E0).");
        }
        // Don't wait for OK, just try to disable
    }
    close_serial_port();
    is_enabled_ = false;
    is_homed_ = false;
    RCLCPP_INFO(logger_, "on_deactivate finished successfully.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn UnoStepperInterface::on_error(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_ERROR(logger_, "on_error called. Closing serial port.");
    close_serial_port();
    is_enabled_ = false;
    is_homed_ = false;
    return CallbackReturn::SUCCESS; // Indicate cleanup was handled
}

CallbackReturn UnoStepperInterface::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
{
     RCLCPP_INFO(logger_, "on_shutdown called. Closing serial port.");
    close_serial_port();
    is_enabled_ = false;
    is_homed_ = false;
    return CallbackReturn::SUCCESS;
}


// --- Hardware Interface Methods ---

std::vector<hardware_interface::StateInterface> UnoStepperInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < num_joints_; ++i)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i])); // Export velocity state
    }
    RCLCPP_INFO(logger_, "Exported %zu state interfaces.", state_interfaces.size());
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> UnoStepperInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < num_joints_; ++i)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    }
     RCLCPP_INFO(logger_, "Exported %zu command interfaces.", command_interfaces.size());
    return command_interfaces;
}

hardware_interface::return_type UnoStepperInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!is_serial_open_) {
        // Don't spam error if port closed intentionally during deactivation/cleanup
        // RCLCPP_ERROR_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 1000, "Serial port not open in read().");
        return hardware_interface::return_type::OK; // Or ERROR? OK prevents controller manager error spam
    }

    // Read lines until buffer is empty or status 'S' is found
    while (true) {
        // Use a very short timeout for polling to avoid blocking the control loop
        std::string line = read_line(std::chrono::milliseconds(1));
        if (line.empty()) {
            // No more complete lines available right now
            break;
        }

        // Log raw lines only in debug if needed
        // RCLCPP_DEBUG(logger_, "Raw serial read: %s", line.c_str());

        if (line.rfind("S ", 0) == 0) { // Check if line starts with "S "
            long steps[4] = {0, 0, 0, 0}; // Assuming Arduino always sends 4 values
            // Use sscanf for simplicity, ensure buffer size is safe if needed
            int parsed_count = sscanf(line.c_str(), "S %ld %ld %ld %ld", &steps[0], &steps[1], &steps[2], &steps[3]);

            if (parsed_count == 4) {
                if (num_joints_ != 4) {
                     RCLCPP_ERROR_ONCE(logger_, "Received 4 step values from Arduino but expected %zu joints!", num_joints_);
                     // Decide how to handle mismatch - ignore? error? use subset?
                     // For now, let's assume the first num_joints_ values correspond
                }

                // Convert steps to radians/meters and update state interfaces
                // Assuming joint order: base, shoulder, elbow, gripper
                size_t joints_to_update = std::min(num_joints_, static_cast<size_t>(4));

                for (size_t i = 0; i < joints_to_update; ++i) {
                    if (i < 3) { // Assuming first 3 are arm joints
                        hw_states_positions_[i] = steps_to_radians(steps[i], i);
                    } else { // Assuming the last joint(s) are gripper(s)
                        // Handle gripper (assuming index 3 maps to the gripper joint)
                        if (joint_names_[i] == "left_finger_joint") { // Be more specific if possible
                             hw_states_positions_[i] = gripper_steps_to_meters(steps[i]);
                        } else {
                             RCLCPP_WARN_ONCE(logger_, "Joint %s at index %zu is not the expected gripper, applying radian conversion.", joint_names_[i].c_str(), i);
                             hw_states_positions_[i] = steps_to_radians(steps[i], i);
                        }
                    }
                    hw_states_velocities_[i] = 0.0; // Assuming no velocity feedback from Arduino
                }

                // Log updated states only in debug
                // RCLCPP_DEBUG(logger_, "States updated: [%.3f, %.3f, %.3f, %.3f]", hw_states_positions_[0], hw_states_positions_[1], hw_states_positions_[2], hw_states_positions_[3]);

                // Found and processed status message, can return OK for this cycle
                return hardware_interface::return_type::OK;
            } else {
                RCLCPP_WARN(logger_, "Failed to parse status line (expected 4 values): '%s'", line.c_str());
            }
        } else if (line.rfind("INFO:", 0) == 0 || line.rfind("DEBUG:", 0) == 0) {
             RCLCPP_INFO(logger_, "Arduino Info/Debug: %s", line.c_str());
        } else if (line.rfind("ERROR:", 0) == 0) {
             RCLCPP_ERROR(logger_, "Arduino Error: %s", line.c_str());
        } else if (line == "OK") {
            // Ignore OK messages here, handled synchronously elsewhere (like in wait_for_ok)
        } else if (!line.empty()) { // Avoid logging empty lines from timeouts
             RCLCPP_WARN(logger_, "Received unexpected line: '%s'", line.c_str());
        }
    }

    // If no 'S' message was processed in this cycle, still return OK
    return hardware_interface::return_type::OK;
}


hardware_interface::return_type UnoStepperInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
     if (!is_serial_open_) {
         RCLCPP_ERROR_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 1000, "Write called but serial port is not open.");
         return hardware_interface::return_type::ERROR;
     }
     if (!is_homed_) {
          RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 1000, "Write called but robot is not homed.");
          // Allow writing even if not homed? Or return ERROR? Let's allow for now.
          // return hardware_interface::return_type::ERROR;
     }
      if (!is_enabled_) {
          RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 1000, "Write called but robot is not enabled.");
          // Allow writing even if not enabled? Or return ERROR? Let's allow for now.
          // return hardware_interface::return_type::ERROR;
     }

    // Convert commanded positions (radians/meters) to steps
    // Ensure we send exactly 4 step values as expected by Arduino
    long target_steps[4] = {0, 0, 0, 0};

    if (num_joints_ != 4) {
        RCLCPP_ERROR_ONCE(logger_, "Expected 4 joints but configured for %zu. Command format might be incorrect for Arduino.", num_joints_);
        // Decide how to handle - error out? Send partial? Send defaults?
        // Let's try sending based on configured joints, padding with last known/zero for others.
    }

    size_t joints_to_command = std::min(num_joints_, static_cast<size_t>(4));
    for (size_t i = 0; i < joints_to_command; ++i) {
         if (i < 3) { // Assuming first 3 are arm joints
            target_steps[i] = radians_to_steps(hw_commands_positions_[i], i);
        } else { // Assuming the last joint(s) are gripper(s)
            // Handle gripper (assuming index 3 maps to the gripper joint)
            if (joint_names_[i] == "left_finger_joint") { // Be more specific if possible
                 target_steps[i] = meters_to_gripper_steps(hw_commands_positions_[i]);
            } else {
                 RCLCPP_WARN_ONCE(logger_, "Joint %s at index %zu is not the expected gripper, applying radian->step conversion for command.", joint_names_[i].c_str(), i);
                 target_steps[i] = radians_to_steps(hw_commands_positions_[i], i);
            }
        }
    }
    // If num_joints_ < 4, the remaining target_steps will be 0.

    // Format the command string: P <base> <shoulder> <elbow> <gripper>\n
    std::stringstream ss;
    ss << "P " << target_steps[0] << " " << target_steps[1] << " " << target_steps[2] << " " << target_steps[3] << "\n";
    std::string command = ss.str();

    // Log command only in debug
    // RCLCPP_DEBUG(logger_, "Sending command: %s", command.c_str());

    // Send the command over serial
    if (!send_command(command)) {
        RCLCPP_ERROR(logger_, "Failed to send command: %s", command.c_str());
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

// --- Helper Methods ---

bool UnoStepperInterface::open_serial_port() {
    if (is_serial_open_) {
        RCLCPP_WARN(logger_, "Serial port %s already open.", serial_port_name_.c_str());
        return true;
    }
    RCLCPP_INFO(logger_, "Opening serial port: %s at baud rate: %d", serial_port_name_.c_str(), baud_rate_);
    boost::system::error_code ec;

    // Ensure io_context is reset if reused
    io_context_.restart();

    serial_port_.open(serial_port_name_, ec);
    if (ec) {
        RCLCPP_FATAL(logger_, "Failed to open serial port %s: %s", serial_port_name_.c_str(), ec.message().c_str());
        return false;
    }

    // Set serial port options
    try {
        serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
        serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
        serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    } catch (const boost::system::system_error& e) {
         RCLCPP_ERROR(logger_, "Failed to set serial port options: %s", e.what());
         serial_port_.close(); // Close port if options failed
         return false;
    }


    // Clear any old data in buffer after opening
    // Use a short timed read to avoid blocking indefinitely if no data arrives
    RCLCPP_INFO(logger_, "Attempting to clear serial buffer...");
    std::string temp_read;
    auto start_time = std::chrono::steady_clock::now();
    while(std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(50)) {
        temp_read = read_line(std::chrono::milliseconds(5)); // Short read attempt
        if (!temp_read.empty()) {
             RCLCPP_DEBUG(logger_, "Cleared from buffer: %s", temp_read.c_str());
        } else {
            break; // No more data quickly available
        }
    }
     RCLCPP_INFO(logger_, "Finished clearing buffer attempt.");


    is_serial_open_ = true;
    RCLCPP_INFO(logger_, "Serial port opened successfully.");
    return true;
}

void UnoStepperInterface::close_serial_port() {
    if (!is_serial_open_) {
        // RCLCPP_DEBUG(logger_, "Serial port %s already closed.", serial_port_name_.c_str());
        return;
    }
    RCLCPP_INFO(logger_, "Closing serial port: %s", serial_port_name_.c_str());
    boost::system::error_code ec;
    serial_port_.cancel(ec); // Cancel any pending async operations
    if (ec) {
         RCLCPP_WARN(logger_, "Error cancelling serial port operations: %s", ec.message().c_str());
    }
    serial_port_.close(ec);
    if (ec) {
        RCLCPP_ERROR(logger_, "Failed to close serial port: %s", ec.message().c_str());
    }
    is_serial_open_ = false;
     RCLCPP_INFO(logger_, "Serial port closed.");
}

bool UnoStepperInterface::send_command(const std::string& command) {
    if (!is_serial_open_) {
        RCLCPP_ERROR(logger_, "Cannot send command, serial port not open.");
        return false;
    }
    boost::system::error_code ec;
    boost::asio::write(serial_port_, boost::asio::buffer(command), ec);
    if (ec) {
        RCLCPP_ERROR(logger_, "Failed to write to serial port: %s", ec.message().c_str());
        // Consider attempting to close/reopen or entering error state
        close_serial_port(); // Close port on write error
        return false;
    }
    return true;
}

// Reads a line ending in '\n' with a timeout using asynchronous operations
std::string UnoStepperInterface::read_line(const std::chrono::milliseconds& timeout) {
    if (!is_serial_open_) return "";

    std::string line;
    boost::system::error_code ec;
    std::size_t bytes_transferred = 0;

    // Set up an asynchronous read operation with a deadline timer
    boost::asio::async_read_until(serial_port_, serial_buffer_, '\n',
        [&](const boost::system::error_code& error, std::size_t bytes) {
            ec = error;
            bytes_transferred = bytes;
        });

    // Run the IO context with a timeout
    // This runs handlers that are ready and returns after timeout if none complete.
    io_context_.restart(); // Restart context before running again
    io_context_.run_for(timeout);

    // Check if the read operation completed (handler was called)
    if (bytes_transferred > 0 && !ec) {
        // Read completed successfully, extract the line
        std::istream is(&serial_buffer_);
        std::getline(is, line);
        // Remove potential '\r' if present before '\n'
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
    } else {
        // Operation did not complete within the timeout or an error occurred
        serial_port_.cancel(ec); // Cancel the pending async operation

        // Check the error code set by the handler or cancellation
        if (ec && ec != boost::asio::error::operation_aborted && ec != boost::asio::error::eof) {
            RCLCPP_ERROR(logger_, "Serial read error: %s", ec.message().c_str());
            close_serial_port(); // Close port on significant read error
        } else if (ec == boost::asio::error::eof) {
            RCLCPP_WARN(logger_, "Serial read EOF - port closed?");
            close_serial_port();
        }
        // On timeout (operation_aborted due to cancel after run_for), return empty string
        line = "";
    }

    return line;
}


bool UnoStepperInterface::wait_for_ok(const std::chrono::milliseconds& timeout) {
     RCLCPP_DEBUG(logger_, "Waiting for OK (timeout: %ld ms)...", timeout.count());
     auto start_time = std::chrono::steady_clock::now();
     while (std::chrono::steady_clock::now() - start_time < timeout) {
         // Calculate remaining time for read_line timeout
         auto time_elapsed = std::chrono::steady_clock::now() - start_time;
         auto time_remaining = timeout - time_elapsed;
         if (time_remaining <= std::chrono::milliseconds(0)) {
             break; // Timeout reached
         }
         // Use a smaller polling timeout for read_line, e.g., 50ms or remaining time if less
         auto read_timeout = std::min(std::chrono::milliseconds(50),
                                      std::chrono::duration_cast<std::chrono::milliseconds>(time_remaining));

         std::string line = read_line(read_timeout);
         if (line == "OK") {
             RCLCPP_DEBUG(logger_, "OK received.");
             return true;
         } else if (!line.empty()) {
             // Log unexpected messages while waiting
             RCLCPP_WARN(logger_, "Unexpected message while waiting for OK: %s", line.c_str());
             // Handle specific messages if needed (e.g., ERROR from Arduino)
             if (line.rfind("ERROR:", 0) == 0) {
                 RCLCPP_ERROR(logger_, "Received ERROR from Arduino while waiting for OK.");
                 return false; // Treat Arduino error as failure
             }
         }
         // No need for extra sleep, read_line handles the polling delay
     }
     RCLCPP_WARN(logger_, "Timeout waiting for OK.");
     return false; // Timeout occurred
}


// --- Conversion Logic ---

long UnoStepperInterface::radians_to_steps(double radians, int joint_index) {
    if (joint_index < 0 || joint_index >= static_cast<int>(num_joints_)) {
        RCLCPP_ERROR(logger_, "Invalid joint index %d in radians_to_steps", joint_index);
        return 0; // Or throw exception
    }
    if (steps_per_rev_ <= 0 || microstepping_ <= 0 || gear_ratios_[joint_index] <= 0) {
         RCLCPP_ERROR_ONCE(logger_, "Invalid conversion parameters (steps/rev, microstep, gear ratio <= 0) for joint %d", joint_index);
         return home_offsets_steps_[joint_index]; // Return offset to avoid large incorrect values
    }
    // Formula: steps = (radians * steps_per_rev * gear_ratio * microstepping) / (2 * PI) + home_offset
    double steps_float = (radians * steps_per_rev_ * gear_ratios_[joint_index] * microstepping_) / (2.0 * M_PI);
    return static_cast<long>(round(steps_float)) + home_offsets_steps_[joint_index];
}

double UnoStepperInterface::steps_to_radians(long steps, int joint_index) {
     if (joint_index < 0 || joint_index >= static_cast<int>(num_joints_)) {
        RCLCPP_ERROR(logger_, "Invalid joint index %d in steps_to_radians", joint_index);
        return 0.0; // Or throw exception
    }
     if (steps_per_rev_ <= 0 || microstepping_ <= 0 || gear_ratios_[joint_index] <= 0) {
         RCLCPP_ERROR_ONCE(logger_, "Invalid conversion parameters (steps/rev, microstep, gear ratio <= 0) for joint %d", joint_index);
         return 0.0; // Return 0 to avoid division by zero
    }
    // Formula: radians = ((steps - home_offset) * 2 * PI) / (steps_per_rev * gear_ratio * microstepping)
    double radians = ((static_cast<double>(steps) - home_offsets_steps_[joint_index]) * 2.0 * M_PI) /
                     (steps_per_rev_ * gear_ratios_[joint_index] * microstepping_);
    return radians;
}

long UnoStepperInterface::meters_to_gripper_steps(double meters) {
    if (gripper_range_meters_ <= 0 || gripper_range_steps_ <= 0) {
        RCLCPP_ERROR_ONCE(logger_, "Gripper range parameters invalid (<= 0). Cannot convert meters to steps.");
        return 0; // Assuming 0 steps is a safe default (e.g., fully closed/open)
    }
    // Linear scaling: steps = (meters / range_meters) * range_steps
    // Clamp meters to the valid range [0, gripper_range_meters_]
    meters = std::max(0.0, std::min(meters, gripper_range_meters_));
    double steps_float = (meters / gripper_range_meters_) * gripper_range_steps_;
    // Add gripper home offset if applicable (assuming gripper uses index 3 for offset)
    long gripper_offset = (num_joints_ > 3) ? home_offsets_steps_[3] : 0;
    return static_cast<long>(round(steps_float)) + gripper_offset;
}

double UnoStepperInterface::gripper_steps_to_meters(long steps) {
    if (gripper_range_meters_ <= 0 || gripper_range_steps_ <= 0) {
         RCLCPP_ERROR_ONCE(logger_, "Gripper range parameters invalid (<= 0). Cannot convert steps to meters.");
        return 0.0; // Assuming 0 meters is a safe default
    }
    // Subtract gripper home offset if applicable (assuming gripper uses index 3 for offset)
    long gripper_offset = (num_joints_ > 3) ? home_offsets_steps_[3] : 0;
    long relative_steps = steps - gripper_offset;

    // Linear scaling: meters = (relative_steps / range_steps) * range_meters
    // Clamp relative_steps to the valid range [0, gripper_range_steps_]
    relative_steps = std::max(0L, std::min(relative_steps, gripper_range_steps_));
    double meters = (static_cast<double>(relative_steps) / gripper_range_steps_) * gripper_range_meters_;
    return meters;
}


} // namespace uno_stepper_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    uno_stepper_interface::UnoStepperInterface,
    hardware_interface::SystemInterface)