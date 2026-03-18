#include "manager/task/launch_preparation_task.hpp"

#include <gtest/gtest.h>

#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager::test {

class LaunchPreparationTaskTest : public ::testing::Test {
protected:
    struct Harness {
        explicit Harness(LaunchPreparationTask::Mode mode)
            : task(
                  belt_command, belt_target_velocity, belt_torque_limit, belt_hold_torque,
                  belt_wait_zero_velocity, left_belt_velocity, right_belt_velocity,
                  left_belt_torque, right_belt_torque, trigger_lock_enable, lifting_command,
                  lifting_left_vel_fb, lifting_right_vel_fb, 1.0, 1, 0, 2000, mode) {}

        rmcs_msgs::DartSliderStatus belt_command{rmcs_msgs::DartSliderStatus::WAIT};
        double belt_target_velocity{0.0};
        double belt_torque_limit{0.0};
        double belt_hold_torque{0.0};
        bool belt_wait_zero_velocity{false};
        double left_belt_velocity{0.0};
        double right_belt_velocity{0.0};
        double left_belt_torque{0.0};
        double right_belt_torque{0.0};
        bool trigger_lock_enable{false};

        rmcs_msgs::DartSliderStatus lifting_command{rmcs_msgs::DartSliderStatus::WAIT};
        double lifting_left_vel_fb{0.0};
        double lifting_right_vel_fb{0.0};

        LaunchPreparationTask task;
    };
};

TEST_F(LaunchPreparationTaskTest, FirstFillStartsWithLiftReset) {
    Harness harness(LaunchPreparationTask::Mode::FIRST_FILL);
    harness.lifting_left_vel_fb = 2.0;
    harness.lifting_right_vel_fb = 2.0;

    EXPECT_EQ(harness.task.tick_first(), ActionStatus::RUNNING);
    EXPECT_EQ(harness.lifting_command, rmcs_msgs::DartSliderStatus::UP);
    EXPECT_EQ(harness.belt_command, rmcs_msgs::DartSliderStatus::WAIT);
    EXPECT_DOUBLE_EQ(harness.belt_target_velocity, 0.0);
    EXPECT_FALSE(harness.trigger_lock_enable);
}

TEST_F(LaunchPreparationTaskTest, FirstFillMovesBeltDownAfterLiftResetCompletes) {
    Harness harness(LaunchPreparationTask::Mode::FIRST_FILL);
    harness.lifting_left_vel_fb = 2.0;
    harness.lifting_right_vel_fb = 2.0;

    EXPECT_EQ(harness.task.tick_first(), ActionStatus::RUNNING);
    EXPECT_EQ(harness.lifting_command, rmcs_msgs::DartSliderStatus::UP);

    harness.lifting_left_vel_fb = 0.0;
    harness.lifting_right_vel_fb = 0.0;
    EXPECT_EQ(harness.task.tick(), ActionStatus::RUNNING);
    EXPECT_EQ(harness.lifting_command, rmcs_msgs::DartSliderStatus::WAIT);
    EXPECT_EQ(harness.belt_command, rmcs_msgs::DartSliderStatus::WAIT);

    EXPECT_EQ(harness.task.tick(), ActionStatus::RUNNING);
    EXPECT_EQ(harness.belt_command, rmcs_msgs::DartSliderStatus::DOWN);
    EXPECT_DOUBLE_EQ(harness.belt_target_velocity, 10.0);
    EXPECT_EQ(harness.lifting_command, rmcs_msgs::DartSliderStatus::WAIT);
}

TEST_F(LaunchPreparationTaskTest, NormalModeStartsWithBeltMove) {
    Harness harness(LaunchPreparationTask::Mode::NORMAL);

    EXPECT_EQ(harness.task.tick_first(), ActionStatus::RUNNING);
    EXPECT_EQ(harness.belt_command, rmcs_msgs::DartSliderStatus::DOWN);
    EXPECT_DOUBLE_EQ(harness.belt_target_velocity, 10.0);
    EXPECT_EQ(harness.lifting_command, rmcs_msgs::DartSliderStatus::WAIT);
    EXPECT_FALSE(harness.trigger_lock_enable);
}

} // namespace rmcs_dart_guidance::manager::test
