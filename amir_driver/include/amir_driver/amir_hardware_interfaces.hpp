#ifndef AMIR_DRIVER__AMIR_HARDWARE_INTERFACES_HPP_
#define AMIR_DRIVER__AMIR_HARDWARE_INTERFACES_HPP_

#include <chrono>
#include <string>
#include <vector>

#include "amir_driver/visibility_control.h"
#include "amir_driver/velocity_command_modes.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "amir_interfaces/msg/amir_cmd.hpp"
#include "amir_interfaces/msg/amir_sensor.hpp"
#include "std_msgs/msg/bool.hpp"

namespace amir_driver
{
class AmirHardwareInterfaces : public hardware_interface::SystemInterface
{
public:
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

private:
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  VelocityCommandModes velocity_modes_;

  // topic based
  rclcpp::Subscription<amir_interfaces::msg::AmirSensor>::SharedPtr amir_joint_states_subscriber_;
  rclcpp::Publisher<amir_interfaces::msg::AmirCmd>::SharedPtr amir_joint_commands_publisher_;
  rclcpp::Node::SharedPtr node_;

  amir_interfaces::msg::AmirSensor latest_joint_state_;
  bool received_joint_state_{false};
  bool initialized_measured_commands_{false};
  bool qp_stop_requested_{false};
  bool received_qp_heartbeat_{false};
  std::chrono::steady_clock::time_point last_qp_heartbeat_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr qp_stop_subscriber_;
};

}  // namespace amir_driver

#endif  // AMIR_DRIVER__AMIR_HARDWARE_INTERFACES_HPP_
