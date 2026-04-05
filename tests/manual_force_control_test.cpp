#include "manager/action/manual_force_control.hpp"

#include <gtest/gtest.h>

#include <eigen3/Eigen/Dense>

namespace rmcs_dart_guidance::manager::test {

TEST(DartManualForceControlActionTest, OutputsScaledVelocityFromJoystick) {
    double force_control_velocity = 0.0;
    Eigen::Vector2d joystick_right(0.4, 0.0);

    DartManualForceControlAction action(force_control_velocity, joystick_right, 5.0, 3.0);

    EXPECT_EQ(action.tick_first(), ActionStatus::RUNNING);
    EXPECT_DOUBLE_EQ(force_control_velocity, 6.0);

    action.on_exit();
    EXPECT_DOUBLE_EQ(force_control_velocity, 0.0);
}

TEST(DartManualForceControlActionTest, ReturnsSuccessAfterConfirmedStall) {
    double force_control_velocity = 0.0;
    Eigen::Vector2d joystick_right(1.0, 0.0);
    double measured_velocity = 0.0;
    double measured_torque = 0.8;

    DartManualForceControlAction action(
        force_control_velocity, joystick_right, 5.0, 3.0, &measured_velocity, &measured_torque,
        0.15, 0.5, 3, 0);

    EXPECT_EQ(action.tick_first(), ActionStatus::RUNNING);
    EXPECT_DOUBLE_EQ(force_control_velocity, 15.0);

    EXPECT_EQ(action.tick(), ActionStatus::RUNNING);
    EXPECT_EQ(action.tick(), ActionStatus::SUCCESS);

    action.on_exit();
    EXPECT_DOUBLE_EQ(force_control_velocity, 0.0);
}

TEST(DartManualForceControlActionTest, ResetsStallCounterWhenCommandDropsToZero) {
    double force_control_velocity = 0.0;
    Eigen::Vector2d joystick_right(1.0, 0.0);
    double measured_velocity = 0.0;
    double measured_torque = 0.8;

    DartManualForceControlAction action(
        force_control_velocity, joystick_right, 5.0, 3.0, &measured_velocity, &measured_torque,
        0.15, 0.5, 3, 0);

    EXPECT_EQ(action.tick_first(), ActionStatus::RUNNING);
    EXPECT_EQ(action.tick(), ActionStatus::RUNNING);

    joystick_right.x() = 0.0;
    EXPECT_EQ(action.tick(), ActionStatus::RUNNING);
    EXPECT_DOUBLE_EQ(force_control_velocity, 0.0);

    joystick_right.x() = 1.0;
    EXPECT_EQ(action.tick(), ActionStatus::RUNNING);
    EXPECT_EQ(action.tick(), ActionStatus::RUNNING);
    EXPECT_EQ(action.tick(), ActionStatus::SUCCESS);
}

} // namespace rmcs_dart_guidance::manager::test
