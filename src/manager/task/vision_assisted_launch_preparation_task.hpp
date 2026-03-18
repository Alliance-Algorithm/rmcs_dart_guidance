#pragma once

#include "manager/action/auto_aim_action.hpp"
#include "manager/task/launch_preparation_task.hpp"
#include "manager/task/task.hpp"

#include <cstdint>
#include <memory>

#include <eigen3/Eigen/Dense>
#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>
#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

// VisionAssistedLaunchPreparationTask:
//   1. 先执行视觉辅助预瞄
//   2. 预瞄成功或超时后，再进入原有机械发射准备流程
class VisionAssistedLaunchPreparationTask : public Task {
public:
    VisionAssistedLaunchPreparationTask(
        rmcs_msgs::DartSliderStatus& belt_command, double& belt_target_velocity,
        double& belt_torque_limit, double& belt_hold_torque, bool& belt_wait_zero_velocity,
        const double& left_belt_velocity, const double& right_belt_velocity,
        const double& left_belt_torque, const double& right_belt_torque, bool& trigger_lock_enable,
        rmcs_msgs::DartSliderStatus& lifting_command, const double& lifting_left_vel_fb,
        const double& lifting_right_vel_fb, double lifting_stall_threshold,
        uint64_t lifting_stall_confirm_ticks, uint64_t lifting_stall_min_run_ticks,
        uint64_t lifting_stall_timeout_ticks, LaunchPreparationTask::Mode mode,
        Eigen::Vector2d& yaw_pitch_control_velocity, bool& aim_ready, Eigen::Vector2d& aim_error_px,
        Eigen::Vector2d& desired_target_px_output, const cv::Point2i& target_position,
        const bool& target_tracking, const rclcpp::Logger& logger, AutoAimParams auto_aim_params)
        : Task("launch_preparation", "视觉辅助滑块发射准备") {

        then(std::make_shared<AutoAimAction>(
            yaw_pitch_control_velocity, aim_ready, aim_error_px, desired_target_px_output,
            target_position, target_tracking, logger, std::move(auto_aim_params)));

        then(std::make_shared<LaunchPreparationTask>(
            belt_command, belt_target_velocity, belt_torque_limit, belt_hold_torque,
            belt_wait_zero_velocity, left_belt_velocity, right_belt_velocity, left_belt_torque,
            right_belt_torque, trigger_lock_enable, lifting_command, lifting_left_vel_fb,
            lifting_right_vel_fb, lifting_stall_threshold, lifting_stall_confirm_ticks,
            lifting_stall_min_run_ticks, lifting_stall_timeout_ticks, mode));
    }
};

} // namespace rmcs_dart_guidance::manager
