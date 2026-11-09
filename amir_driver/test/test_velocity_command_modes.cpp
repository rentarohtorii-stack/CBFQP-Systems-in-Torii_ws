#include <cmath>
#include <limits>
#include <gtest/gtest.h>
#include "amir_driver/velocity_command_modes.hpp"
#include "amir_driver/command_validity.hpp"

TEST(VelocityCommandModes, PositionOnlyIgnoresUnclaimedNaN)
{
  amir_driver::VelocityCommandModes modes;
  modes.switch_interfaces({"Joint_2/position", "Gripper/position"}, {});
  EXPECT_DOUBLE_EQ(modes.velocity("Joint_2", std::numeric_limits<double>::quiet_NaN()), 0.0);
  EXPECT_DOUBLE_EQ(modes.velocity("Gripper", 0.3), 0.0);
}

TEST(VelocityCommandModes, TrajectoryToPositionDiscardsStaleVelocityAndCanSwitchBack)
{
  amir_driver::VelocityCommandModes modes;
  modes.switch_interfaces({"Joint_2/position", "Joint_2/velocity", "Joint_3/velocity"}, {});
  EXPECT_DOUBLE_EQ(modes.velocity("Joint_2", -0.12), -0.12);
  EXPECT_DOUBLE_EQ(modes.velocity("Joint_3", 0.2), 0.2);
  modes.switch_interfaces({"Joint_2/position"}, {"Joint_2/position", "Joint_2/velocity"});
  EXPECT_DOUBLE_EQ(modes.velocity("Joint_2", -0.12), 0.0);
  EXPECT_DOUBLE_EQ(modes.velocity("Joint_3", 0.2), 0.2);
  modes.switch_interfaces({"Joint_2/position", "Joint_2/velocity"}, {"Joint_2/position"});
  EXPECT_DOUBLE_EQ(modes.velocity("Joint_2", 0.08), 0.08);
  EXPECT_TRUE(std::isnan(modes.velocity("Joint_2", std::numeric_limits<double>::quiet_NaN())));
}

TEST(CommandValidity, GripperWithoutGoalDoesNotBlockValidArmCommands)
{
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(amir_driver::command_validity("Gripper", nan, 0.0),
    amir_driver::CommandValidity::HoldJoint);
  EXPECT_EQ(amir_driver::command_validity("Joint_2", 1.0, 0.2),
    amir_driver::CommandValidity::Forward);
  EXPECT_EQ(amir_driver::command_validity("Gripper", -0.6, 0.0),
    amir_driver::CommandValidity::Forward);
}

TEST(CommandValidity, InvalidArmOrNonFiniteVelocityStillStopsAllJoints)
{
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  const auto inf = std::numeric_limits<double>::infinity();
  EXPECT_EQ(amir_driver::command_validity("Joint_2", nan, 0.2),
    amir_driver::CommandValidity::HoldAll);
  EXPECT_EQ(amir_driver::command_validity("Joint_3", 1.0, inf),
    amir_driver::CommandValidity::HoldAll);
  EXPECT_EQ(amir_driver::command_validity("Gripper", inf, 0.0),
    amir_driver::CommandValidity::HoldAll);
  EXPECT_EQ(amir_driver::command_validity("Gripper", nan, nan),
    amir_driver::CommandValidity::HoldAll);
}
