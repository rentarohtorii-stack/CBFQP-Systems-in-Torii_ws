#ifndef AMIR_DRIVER__VELOCITY_COMMAND_MODES_HPP_
#define AMIR_DRIVER__VELOCITY_COMMAND_MODES_HPP_

#include <algorithm>
#include <string>
#include <vector>

namespace amir_driver
{
// Track velocity ownership separately from the value left in command storage.
// A position-only controller must not reuse a previous trajectory's velocity.
class VelocityCommandModes
{
public:
  void switch_interfaces(
    const std::vector<std::string> & start, const std::vector<std::string> & stop)
  {
    for (const auto & name : stop) {
      active_.erase(std::remove(active_.begin(), active_.end(), name), active_.end());
    }
    for (const auto & name : start) {
      if (std::find(active_.begin(), active_.end(), name) == active_.end()) {
        active_.push_back(name);
      }
    }
  }

  double velocity(const std::string & joint_name, double stored_command) const
  {
    const auto interface_name = joint_name + "/velocity";
    return std::find(active_.begin(), active_.end(), interface_name) != active_.end() ?
           stored_command : 0.0;
  }

private:
  std::vector<std::string> active_;
};
}  // namespace amir_driver

#endif  // AMIR_DRIVER__VELOCITY_COMMAND_MODES_HPP_
