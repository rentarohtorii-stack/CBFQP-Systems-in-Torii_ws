#ifndef AMIR_DRIVER__COMMAND_VALIDITY_HPP_
#define AMIR_DRIVER__COMMAND_VALIDITY_HPP_

#include <cmath>
#include <string>

namespace amir_driver
{
enum class CommandValidity {Forward, HoldJoint, HoldAll};

inline CommandValidity command_validity(
  const std::string & joint_name, double position, double velocity)
{
  if (!std::isfinite(velocity)) {
    return CommandValidity::HoldAll;
  }
  if (std::isfinite(position)) {
    return CommandValidity::Forward;
  }
  // A position-only gripper controller can leave NaN until its first goal.
  // Hold this joint at its measured angle instead of blocking the arm.
  if (joint_name == "Gripper" && std::isnan(position)) {
    return CommandValidity::HoldJoint;
  }
  return CommandValidity::HoldAll;
}
}  // namespace amir_driver

#endif  // AMIR_DRIVER__COMMAND_VALIDITY_HPP_
