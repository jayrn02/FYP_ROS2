#ifndef UNO_STEPPER_INTERFACE__UNO_STEPPER_INTERFACE_HPP_
#define UNO_STEPPER_INTERFACE__UNO_STEPPER_INTERFACE_HPP_

#include <vector>
#include <string>
#include <chrono> // For timeouts

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

// Include Boost.Asio headers
#include <boost/asio/serial_port.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/streambuf.hpp>

namespace uno_stepper_interface
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class UnoStepperInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(UnoStepperInterface)

    // --- Constructor ---
    UnoStepperInterface(); // Declaration only

    // --- Lifecycle Methods ---
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override; // Declaration only
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override; // Declaration only
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override; // Declaration only
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override; // Declaration only
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override; // Declaration only
    CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override; // Declaration only
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override; // Declaration only

    // --- Hardware Interface Methods ---
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override; // Declaration only
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override; // Declaration only
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override; // Declaration only
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override; // Declaration only

private:
    // --- Helper Methods ---
    bool open_serial_port(); // Declaration only
    void close_serial_port(); // Declaration only
    bool send_command(const std::string& command); // Declaration only
    std::string read_line(const std::chrono::milliseconds& timeout); // Declaration only
    bool wait_for_ok(const std::chrono::milliseconds& timeout); // Declaration only

    // --- Conversion Logic ---
    long radians_to_steps(double radians, int joint_index); // Declaration only
    double steps_to_radians(long steps, int joint_index); // Declaration only
    long meters_to_gripper_steps(double meters); // Declaration only
    double gripper_steps_to_meters(long steps); // Declaration only

    // --- Member Variables ---
    // *** Order matters for initialization list warnings ***
    // Logger should generally be first or early
    rclcpp::Logger logger_;

    // Parameters from YAML
    std::string serial_port_name_;
    int baud_rate_;
    int serial_timeout_ms_;
    int steps_per_rev_;
    int microstepping_;
    std::vector<double> gear_ratios_;
    std::vector<long> home_offsets_steps_;
    double gripper_range_meters_;
    long gripper_range_steps_;

    // Boost Asio Serial Communication - io_context must come before serial_port
    boost::asio::io_context io_context_;
    boost::asio::serial_port serial_port_;
    boost::asio::streambuf serial_buffer_;

    // Store the command and state interfaces
    std::vector<double> hw_commands_positions_;
    std::vector<double> hw_states_positions_;
    std::vector<double> hw_states_velocities_;

    // Store joint names in order
    std::vector<std::string> joint_names_;
    std::size_t num_joints_;

    // Status flags
    bool is_serial_open_ = false;
    bool is_homed_ = false;
    bool is_enabled_ = false;
};

} // namespace uno_stepper_interface

#endif // UNO_STEPPER_INTERFACE__UNO_STEPPER_INTERFACE_HPP_