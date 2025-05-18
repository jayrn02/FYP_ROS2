#ifndef UNO_STEPPER_INTERFACE__UNO_STEPPER_INTERFACE_HPP_
#define UNO_STEPPER_INTERFACE__UNO_STEPPER_INTERFACE_HPP_

#include <vector>
#include <string>
#include <chrono>
#include <memory> // For unique_ptr

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// <<< ADDED: Boost.Asio includes for serial port >>>
#include <boost/asio/io_context.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/write.hpp>
#include <boost/system/error_code.hpp>

namespace uno_stepper_interface
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class UnoStepperInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(UnoStepperInterface)

    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // <<< MODIFIED: Use Boost.Asio objects directly >>>
    boost::asio::io_context io_context_;
    std::unique_ptr<boost::asio::serial_port> serial_port_; // Use unique_ptr
    boost::asio::streambuf read_asio_buffer_; // Use Asio streambuf for read_until

    std::string serial_port_name_; // Renamed from serial_port_ to avoid conflict
    int32_t baud_rate_;
    std::chrono::milliseconds timeout_ms_;

    // Store hardware info parameters (same as before)
    std::vector<std::string> joint_names_;
    double gear_ratio_;
    double steps_per_rev_;
    double microstepping_;
    std::vector<long> home_steps_;
    double gripper_steps_closed_, gripper_steps_open_;
    double gripper_meters_closed_, gripper_meters_open_;

    // Conversion factors (same as before)
    double rad_to_step_factor_;
    double step_to_rad_factor_;
    double gripper_meters_to_steps_factor_;
    double gripper_steps_to_meters_factor_;

    // Store the state and command for joints (same as before)
    std::vector<double> hw_commands_positions_;
    std::vector<double> hw_states_positions_;
    std::vector<double> hw_states_velocities_;

    std::string last_read_msg_; // Keep for parsed line

    // <<< MODIFIED: Updated helper function signature >>>
    bool sendSerialCommand(const std::string& command, const std::string& wait_for = "", bool log_error = true);
    // Helper function to parse status string (same signature)
    bool parseStatusString(const std::string& status_str);
    // <<< ADDED: Helper to read a line with timeout (example) >>>
    bool readLineWithTimeout(std::string& line, std::chrono::milliseconds timeout);

};

} // namespace uno_stepper_interface

#endif // UNO_STEPPER_INTERFACE__UNO_STEPPER_INTERFACE_HPP_