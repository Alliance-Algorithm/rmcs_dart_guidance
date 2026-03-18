#include "manager/action/auto_aim_action.hpp"

#include <gtest/gtest.h>

#include <cstdlib>

#include <eigen3/Eigen/Dense>
#include <opencv2/core/types.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rmcs_dart_guidance::manager::test {

class AutoAimActionTest : public ::testing::Test {
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

    struct Harness {
        explicit Harness(AutoAimParams params)
            : action(
                  yaw_pitch_control_velocity, aim_ready, aim_error_px, desired_target_px_output,
                  target_position, tracking, rclcpp::get_logger("auto_aim_action_test"),
                  std::move(params)) {}

        ActionStatus tick_first() { return action.tick_first(); }

        ActionStatus tick() { return action.tick(); }

        Eigen::Vector2d yaw_pitch_control_velocity{Eigen::Vector2d::Zero()};
        bool aim_ready{false};
        Eigen::Vector2d aim_error_px{Eigen::Vector2d::Zero()};
        Eigen::Vector2d desired_target_px_output{Eigen::Vector2d::Zero()};
        cv::Point2i target_position{-1, -1};
        bool tracking{false};
        AutoAimAction action;
    };

    static AutoAimParams make_default_params() {
        AutoAimParams params;
        params.desired_target_px = Eigen::Vector2d(100.0, 100.0);
        params.deadband_px = Eigen::Vector2d::Constant(3.0);
        params.ready_exit_deadband_px = Eigen::Vector2d::Constant(5.0);
        params.accept_deadband_px = Eigen::Vector2d::Constant(8.0);
        params.yaw_gain = 0.1;
        params.pitch_gain = 0.1;
        params.ready_confirm_ticks = 1;
        params.timeout_ticks = 4;
        params.min_transform_rate = 0.0;
        params.max_transform_rate = 10.0;
        return params;
    }
};

TEST_F(AutoAimActionTest, StrictReadyReturnsSuccessAndSetsReady) {
    Harness harness(make_default_params());
    harness.tracking = true;
    harness.target_position = cv::Point2i(98, 100);

    EXPECT_EQ(harness.tick_first(), ActionStatus::SUCCESS);
    EXPECT_TRUE(harness.aim_ready);
    EXPECT_TRUE(harness.yaw_pitch_control_velocity.isZero(1e-9));
    EXPECT_DOUBLE_EQ(harness.aim_error_px.x(), 2.0);
    EXPECT_DOUBLE_EQ(harness.aim_error_px.y(), 0.0);
}

TEST_F(AutoAimActionTest, AcceptableErrorReturnsSuccessWithoutReady) {
    Harness harness(make_default_params());
    harness.tracking = true;
    harness.target_position = cv::Point2i(94, 100);

    EXPECT_EQ(harness.tick_first(), ActionStatus::SUCCESS);
    EXPECT_FALSE(harness.aim_ready);
    EXPECT_TRUE(harness.yaw_pitch_control_velocity.isZero(1e-9));
    EXPECT_DOUBLE_EQ(harness.aim_error_px.x(), 6.0);
    EXPECT_DOUBLE_EQ(harness.aim_error_px.y(), 0.0);
}

TEST_F(AutoAimActionTest, UsesMinimumTransformRateOutsideAcceptDeadband) {
    AutoAimParams params = make_default_params();
    params.min_transform_rate = 1.0;

    Harness harness(params);
    harness.tracking = true;
    harness.target_position = cv::Point2i(90, 100);

    EXPECT_EQ(harness.tick_first(), ActionStatus::RUNNING);
    EXPECT_DOUBLE_EQ(harness.yaw_pitch_control_velocity.x(), 1.0);
    EXPECT_DOUBLE_EQ(harness.yaw_pitch_control_velocity.y(), 0.0);
    EXPECT_FALSE(harness.aim_ready);
}

TEST_F(AutoAimActionTest, TimeoutPreservesLastValidErrorAfterTrackingLoss) {
    Harness harness(make_default_params());
    harness.tracking = true;
    harness.target_position = cv::Point2i(90, 100);

    EXPECT_EQ(harness.tick_first(), ActionStatus::RUNNING);
    EXPECT_DOUBLE_EQ(harness.aim_error_px.x(), 10.0);
    EXPECT_DOUBLE_EQ(harness.aim_error_px.y(), 0.0);

    harness.tracking = false;
    harness.target_position = cv::Point2i(-1, -1);

    EXPECT_EQ(harness.tick(), ActionStatus::RUNNING);
    EXPECT_EQ(harness.tick(), ActionStatus::RUNNING);
    EXPECT_EQ(harness.tick(), ActionStatus::SUCCESS);

    EXPECT_FALSE(harness.aim_ready);
    EXPECT_TRUE(harness.yaw_pitch_control_velocity.isZero(1e-9));
    EXPECT_DOUBLE_EQ(harness.aim_error_px.x(), 10.0);
    EXPECT_DOUBLE_EQ(harness.aim_error_px.y(), 0.0);
}

} // namespace rmcs_dart_guidance::manager::test
