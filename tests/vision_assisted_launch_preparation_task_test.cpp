#include "manager/task/vision_assisted_launch_preparation_task.hpp"

#include <gtest/gtest.h>

#include <cstdlib>

#include <eigen3/Eigen/Dense>
#include <opencv2/core/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager::test {

class VisionAssistedLaunchPreparationTaskTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!rclcpp::ok()) {
            ::setenv("ROS_LOG_DIR", "/tmp/rmcs_dart_guidance_test_logs", 1);
            int argc = 0;
            char** argv = nullptr;
            rclcpp::init(argc, argv);
        }
    }

    static void TearDownTestSuite() {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    static AutoAimParams make_auto_aim_params() {
        AutoAimParams params;
        params.desired_target_px = Eigen::Vector2d(100.0, 100.0);
        params.deadband_px = Eigen::Vector2d::Constant(2.0);
        params.ready_exit_deadband_px = Eigen::Vector2d::Constant(3.0);
        params.accept_deadband_px = Eigen::Vector2d::Constant(6.0);
        params.ready_confirm_ticks = 1;
        params.timeout_ticks = 2;
        params.yaw_gain = 0.1;
        params.pitch_gain = 0.1;
        params.max_transform_rate = 10.0;
        return params;
    }

    struct Harness {
        explicit Harness(LaunchPreparationTask::Mode mode)
            : task(
                  belt_command, belt_target_velocity, belt_torque_limit, belt_hold_torque,
                  belt_wait_zero_velocity, left_belt_velocity, right_belt_velocity,
                  left_belt_torque, right_belt_torque, trigger_lock_enable, lifting_command,
                  lifting_left_vel_fb, lifting_right_vel_fb, 1.0, 1, 0, 2000, mode,
                  yaw_pitch_control_velocity, aim_ready, aim_error_px, desired_target_px_output,
                  target_position, target_tracking,
                  rclcpp::get_logger("vision_assisted_launch_preparation_task_test"),
                  make_auto_aim_params()) {}

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

        Eigen::Vector2d yaw_pitch_control_velocity{Eigen::Vector2d::Zero()};
        bool aim_ready{false};
        Eigen::Vector2d aim_error_px{Eigen::Vector2d::Zero()};
        Eigen::Vector2d desired_target_px_output{Eigen::Vector2d::Zero()};
        cv::Point2i target_position{-1, -1};
        bool target_tracking{false};

        VisionAssistedLaunchPreparationTask task;
    };
};

TEST_F(VisionAssistedLaunchPreparationTaskTest, StartsWithAutoAimBeforeMechanicalSequence) {
    Harness harness(LaunchPreparationTask::Mode::NORMAL);
    harness.target_tracking = true;
    harness.target_position = cv::Point2i(90, 100);

    EXPECT_EQ(harness.task.tick_first(), ActionStatus::RUNNING);
    EXPECT_EQ(harness.belt_command, rmcs_msgs::DartSliderStatus::WAIT);
    EXPECT_EQ(harness.lifting_command, rmcs_msgs::DartSliderStatus::WAIT);
    EXPECT_GT(harness.yaw_pitch_control_velocity.x(), 0.0);
    EXPECT_FALSE(harness.aim_ready);
}

TEST_F(VisionAssistedLaunchPreparationTaskTest, TimeoutContinuesIntoMechanicalSequence) {
    Harness harness(LaunchPreparationTask::Mode::NORMAL);

    EXPECT_EQ(harness.task.tick_first(), ActionStatus::RUNNING);
    EXPECT_EQ(harness.belt_command, rmcs_msgs::DartSliderStatus::WAIT);

    EXPECT_EQ(harness.task.tick(), ActionStatus::RUNNING);
    EXPECT_EQ(harness.belt_command, rmcs_msgs::DartSliderStatus::WAIT);

    EXPECT_EQ(harness.task.tick(), ActionStatus::RUNNING);
    EXPECT_EQ(harness.belt_command, rmcs_msgs::DartSliderStatus::DOWN);
    EXPECT_DOUBLE_EQ(harness.belt_target_velocity, 10.0);
}

TEST_F(VisionAssistedLaunchPreparationTaskTest, FirstFillModePreservesLiftResetAfterAim) {
    Harness harness(LaunchPreparationTask::Mode::FIRST_FILL);
    harness.target_tracking = true;
    harness.target_position = cv::Point2i(100, 100);

    EXPECT_EQ(harness.task.tick_first(), ActionStatus::RUNNING);
    EXPECT_TRUE(harness.aim_ready);
    EXPECT_EQ(harness.belt_command, rmcs_msgs::DartSliderStatus::WAIT);

    harness.lifting_left_vel_fb = 2.0;
    harness.lifting_right_vel_fb = 2.0;
    EXPECT_EQ(harness.task.tick(), ActionStatus::RUNNING);
    EXPECT_EQ(harness.lifting_command, rmcs_msgs::DartSliderStatus::UP);
    EXPECT_EQ(harness.belt_command, rmcs_msgs::DartSliderStatus::WAIT);
}

} // namespace rmcs_dart_guidance::manager::test
