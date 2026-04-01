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
        double& belt_torque_offset, const double& left_belt_angle, const double& right_belt_angle,
        const double& left_belt_velocity, const double& right_belt_velocity,
        const double& left_belt_torque, const double& right_belt_torque, bool& trigger_lock_enable,
        double belt_down_distance, double belt_pulley_radius, double down_velocity,
        double torque_limit, double up_torque_limit, uint64_t down_ramp_ticks,
        double down_torque_offset, double down_hold_torque, double down_zero_velocity_threshold,
        uint64_t down_zero_confirm_ticks, uint64_t down_ramp_timeout_ticks,
        bool require_lifting_down, rmcs_msgs::DartSliderStatus& lifting_command,
        const double& lifting_left_vel_fb, const double& lifting_right_vel_fb,
        bool& belt_zero_calibration, double belt_up_distance, double up_velocity,
        double up_decel_target_velocity, double up_decel_torque_offset,
        double up_stall_velocity_threshold, uint64_t up_stall_confirm_ticks,
        uint64_t up_stall_min_run_ticks, uint64_t up_decel_timeout_ticks,
        Eigen::Vector2d& yaw_pitch_control_velocity, bool& aim_ready, Eigen::Vector2d& aim_error_px,
        Eigen::Vector2d& desired_target_px_output, const cv::Point2i& target_position,
        const bool& target_tracking, const rclcpp::Logger& logger, AutoAimParams auto_aim_params,
        bool is_first_shot = false)
        : Task("launch_preparation", "视觉辅助滑块发射准备") {

        then(
            std::make_shared<AutoAimAction>(
                yaw_pitch_control_velocity, aim_ready, aim_error_px, desired_target_px_output,
                target_position, target_tracking, logger, std::move(auto_aim_params)));

        then(
            std::make_shared<LaunchPreparationTask>(
                belt_command, belt_target_velocity, belt_torque_limit, belt_hold_torque,
                belt_wait_zero_velocity, belt_torque_offset, left_belt_angle, right_belt_angle,
                left_belt_velocity, right_belt_velocity, left_belt_torque, right_belt_torque,
                trigger_lock_enable, belt_down_distance, belt_pulley_radius, down_velocity,
                torque_limit, up_torque_limit, down_ramp_ticks, down_torque_offset,
                down_hold_torque, down_zero_velocity_threshold, down_zero_confirm_ticks,
                down_ramp_timeout_ticks, require_lifting_down, lifting_command, lifting_left_vel_fb,
                lifting_right_vel_fb, belt_zero_calibration, belt_up_distance, up_velocity,
                up_decel_target_velocity, up_decel_torque_offset, up_stall_velocity_threshold,
                up_stall_confirm_ticks, up_stall_min_run_ticks, up_decel_timeout_ticks,
                is_first_shot));
    }
};

} // namespace rmcs_dart_guidance::manager
