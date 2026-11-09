#include <cmath>
#include <limits>
#include <vector>

#include "amir_driver/amir_hardware_interfaces.hpp"
#include "amir_driver/command_validity.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#define DEG_TO_MRAD(x) ((x)*(17.453292519943295769236907684886))

namespace amir_driver
{

static rclcpp::Logger LOGGER = rclcpp::get_logger("Amir740Hardware");
const int initial_position[6] = {2970, 2360, -2790, 1310, -2760, 260};  // mrad

hardware_interface::CallbackReturn AmirHardwareInterfaces::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  rclcpp::NodeOptions options;
  options.arguments({ "--ros-args", "-r", "__node:=amir_ros2_control_topic_" + info_.name });

  node_ = rclcpp::Node::make_shared("_", options);

  amir_joint_states_subscriber_ = node_->create_subscription<amir_interfaces::msg::AmirSensor>(
      "encoder_pub", rclcpp::SensorDataQoS(),
      [this](const amir_interfaces::msg::AmirSensor::SharedPtr amir_state) {
        for (size_t i = 0; i < info_.joints.size(); ++i) {
          if (!std::isfinite(amir_state->angle[i]) || !std::isfinite(amir_state->vel[i])) {
            return;
          }
        }
        latest_joint_state_ = *amir_state;
        received_joint_state_ = true;
      });
  amir_joint_commands_publisher_ = node_->create_publisher<amir_interfaces::msg::AmirCmd>(
      "motor_sub", rclcpp::QoS(1));
  qp_stop_subscriber_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/amir/qp_stop", rclcpp::QoS(1),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        qp_stop_requested_ = message->data;
        received_qp_heartbeat_ = true;
        last_qp_heartbeat_ = std::chrono::steady_clock::now();
      });

  RCLCPP_INFO(LOGGER, "Successfully init! QP hold diagnostics enabled (heartbeat timeout: 250 ms)");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AmirHardwareInterfaces::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Successfully configured!");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AmirHardwareInterfaces::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //   info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_states_accelerations_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AmirHardwareInterfaces::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn AmirHardwareInterfaces::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Activating ...please wait...");

  for (size_t i = 0; i < hw_states_positions_.size(); ++i) {
    if (std::isnan(hw_states_velocities_[i])) hw_states_velocities_[i] = 0;

    // command and state should be equal when starting
    hw_commands_positions_[i] = hw_states_positions_[i];
    hw_commands_velocities_[i] = hw_states_velocities_[i];
  }

  RCLCPP_INFO(LOGGER, "Successfully activated!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AmirHardwareInterfaces::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Deactivating ...please wait...");
  // TODO: prepare the robot to stop receiving commands

  RCLCPP_INFO(LOGGER, "Successfully deactivated!");

  return CallbackReturn::SUCCESS;
}


hardware_interface::return_type AmirHardwareInterfaces::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /* period */)
{
  rclcpp::spin_some(node_);

  if (!received_joint_state_) {
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < hw_states_velocities_.size(); i++) {
    hw_states_positions_[i] = (latest_joint_state_.angle[i] + initial_position[i]) / 1000.0; // rad
    hw_states_velocities_[i] = (latest_joint_state_.vel[i]) / 1000.0; // rad/s
  }

  if (!initialized_measured_commands_) {
    hw_commands_positions_ = hw_states_positions_;
    for (auto & velocity : hw_commands_velocities_) velocity = 0.0;
    initialized_measured_commands_ = true;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AmirHardwareInterfaces::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  for (const auto & name : stop_interfaces) {
    RCLCPP_INFO(LOGGER, "Stopping command interface: %s", name.c_str());
  }
  for (const auto & name : start_interfaces) {
    RCLCPP_INFO(LOGGER, "Starting command interface: %s", name.c_str());
  }
  velocity_modes_.switch_interfaces(start_interfaces, stop_interfaces);
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    hw_commands_velocities_[i] = velocity_modes_.velocity(
      info_.joints[i].name, hw_commands_velocities_[i]);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AmirHardwareInterfaces::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /* period */)
{
  static const float tolerance = DEG_TO_MRAD(0.500);
  static const float velocity_buffer = 300.0f;

  if (!initialized_measured_commands_) {
    RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 2000,
      "motor_sub waiting for finite encoder_pub feedback");
    return hardware_interface::return_type::OK;
  }

  // Forward controllers retain their last command: stop if the QP heartbeat disappears.
  const auto qp_heartbeat_age_ms = received_qp_heartbeat_ ?
    std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - last_qp_heartbeat_).count() : -1;
  const bool qp_timed_out = received_qp_heartbeat_ &&
    std::chrono::steady_clock::now() - last_qp_heartbeat_ > std::chrono::milliseconds(250);
  bool invalid_command = false;
  for (size_t i = 0; i < hw_commands_positions_.size(); ++i) {
    // Unclaimed velocity interfaces may contain NaN or an old trajectory value.
    // Position-only control always sends zero for the arm velocity fields.
    const double requested_velocity = hw_commands_velocities_[i];
    hw_commands_velocities_[i] = velocity_modes_.velocity(
      info_.joints[i].name, hw_commands_velocities_[i]);
    if (std::isfinite(requested_velocity) && std::abs(requested_velocity) > 1e-12 &&
      hw_commands_velocities_[i] == 0.0)
    {
      RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 2000,
        "Velocity suppressed for %s: requested=%g rad/s, velocity interface not active in hardware",
        info_.joints[i].name.c_str(), requested_velocity);
    }
    const auto validity = command_validity(
      info_.joints[i].name, hw_commands_positions_[i], hw_commands_velocities_[i]);
    if (validity == CommandValidity::HoldJoint) {
      RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 2000,
        "Gripper position unset (NaN): holding gripper only at measured angle");
    }
    if (validity == CommandValidity::HoldAll) {
      RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 2000,
        "Invalid command for %s: position=%g velocity=%g; holding all joints",
        info_.joints[i].name.c_str(), hw_commands_positions_[i], hw_commands_velocities_[i]);
      invalid_command = true;
    }
  }

  if (qp_stop_requested_ || qp_timed_out || invalid_command) {
    RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 2000,
      "motor_sub HOLD: qp_stop=%d heartbeat_timeout=%d invalid_command=%d heartbeat_age_ms=%lld",
      static_cast<int>(qp_stop_requested_), static_cast<int>(qp_timed_out),
      static_cast<int>(invalid_command), static_cast<long long>(qp_heartbeat_age_ms));
  }

  amir_interfaces::msg::AmirCmd amir_cmd;

  for (size_t i = 0; i < hw_commands_velocities_.size(); i++) {
    if (qp_stop_requested_ || qp_timed_out || invalid_command) {
      // Bypass the minimum-speed fallback and any queued controller target.
      amir_cmd.angle[i] = latest_joint_state_.angle[i];
      amir_cmd.vel[i] = 0.0f;
      hw_commands_positions_[i] = hw_states_positions_[i];
      hw_commands_velocities_[i] = 0.0;
      continue;
    }

    if (command_validity(info_.joints[i].name, hw_commands_positions_[i],
      hw_commands_velocities_[i]) == CommandValidity::HoldJoint)
    {
      // Do not apply the gripper's minimum-speed fallback without a position goal.
      amir_cmd.angle[i] = latest_joint_state_.angle[i];
      amir_cmd.vel[i] = 0.0f;
      continue;
    }

    amir_cmd.angle[i] = static_cast<float>((hw_commands_positions_[i] * 1000.0f) - initial_position[i]); // in mrad
    amir_cmd.vel[i] = static_cast<float>(hw_commands_velocities_[i]) * 1000.0f; // in mrad/s
  
    // Arm velocity is zero in position-only mode; trajectory velocities pass through.
    // Retain the legacy minimum-speed workaround only for the gripper.
    if (info_.joints[i].name == "Gripper" && fabs(latest_joint_state_.angle[i] - amir_cmd.angle[i]) >= tolerance) {
      if (amir_cmd.vel[i] >= 0.0f && amir_cmd.vel[i] < velocity_buffer) {
        amir_cmd.vel[i] = velocity_buffer;
      }
      if (amir_cmd.vel[i] <= 0.0f && amir_cmd.vel[i] >= -velocity_buffer) {
        amir_cmd.vel[i] = -velocity_buffer;
      }
    }
  }

  amir_joint_commands_publisher_->publish(amir_cmd);

  return hardware_interface::return_type::OK;
}

}  // namespace amir_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  amir_driver::AmirHardwareInterfaces, hardware_interface::SystemInterface)
