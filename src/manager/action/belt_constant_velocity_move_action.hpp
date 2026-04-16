#pragma once

#include "action.hpp"

#include <cmath>
#include <cstdint>
#include <string>

#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

// BeltConstantVelocityMoveAction - 匀速运动到目标位置
//   不使用PID位置控制，直接使用速度控制模式
//   完成条件：到达目标位置（基于多圈角度反馈）
//   优点：避免PID error过大导致的问题，运动更平滑
//
//   重要：目标位置基于初始化后的实际角度计算
//   - 输入参数是距离（m），在函数内换算为角度（rad）
//   - 目标位置 = 初始角度 + (目标距离 / 滑轮半径)
class BeltConstantVelocityMoveAction : public IAction {
public:
    BeltConstantVelocityMoveAction(
        std::string name, rmcs_msgs::DartSliderStatus& belt_command, double& belt_target_velocity,
        double& belt_torque_offset, double& belt_torque_limit, double torque_offset_value,
        const double& left_belt_angle, const double& right_belt_angle,
        const double& left_belt_velocity, const double& right_belt_velocity,
        double target_distance,          // 目标距离（m，正=上行，负=下行）
        double velocity,                 // 运动速度（rad/s）
        double torque_limit,             // 扭矩限制（N⋅m）
        uint64_t timeout_ticks,
        uint64_t min_running_ticks = 50) // 下行最大距离限制（m，正值，防止过度下行）
        : IAction(std::move(name))
        , belt_command_(belt_command)
        , belt_target_velocity_(belt_target_velocity)
        , belt_torque_offset_(belt_torque_offset)
        , belt_torque_limit_(belt_torque_limit)
        , torque_offset_value_(torque_offset_value)
        , left_belt_angle_(left_belt_angle)
        , right_belt_angle_(right_belt_angle)
        , left_belt_velocity_(left_belt_velocity)
        , right_belt_velocity_(right_belt_velocity)
        , target_distance_(target_distance)
        , velocity_(velocity)
        , torque_limit_(torque_limit)
        , timeout_ticks_(timeout_ticks)
        , min_running_ticks_(min_running_ticks) {}

    void on_enter() override {
        stall_counter_ = 0;
        initial_angle_ = left_belt_angle_;
        double target_angle_offset =
            target_distance_ / 0.0195;   // 将目标距离换算为角度偏移（rad），滑轮半径为0.0195m
        target_position_ = initial_angle_ + target_angle_offset;

        double distance_to_target = target_position_ - initial_angle_;
        if (distance_to_target > 0) {
            belt_command_ = rmcs_msgs::DartSliderStatus::DOWN;
            belt_target_velocity_ = std::abs(velocity_);
        } else {
            belt_command_ = rmcs_msgs::DartSliderStatus::UP;
            belt_target_velocity_ = std::abs(velocity_);
        }

        belt_torque_offset_ = torque_offset_value_;
        belt_torque_limit_ = torque_limit_;
    }

    ActionStatus update() override {
        if (elapsed_ticks() >= timeout_ticks_) {
            return ActionStatus::FAILURE;
        }

        double avg_angle = (left_belt_angle_ + right_belt_angle_) / 2.0;
        double position_error = target_position_ - avg_angle; // 保留符号，用于判断方向
        double avg_velocity =
            (std::abs(left_belt_velocity_) + std::abs(right_belt_velocity_)) / 2.0;

        // 位置到达判断（优先级最高）
        if (elapsed_ticks() > min_running_ticks_) {
            bool within_tolerance = std::abs(position_error) < 0.01;
            bool overshot = false;

            if (target_distance_ > 0) {
                overshot = (position_error < 0);
            } else {
                overshot = (position_error > 0);
            }

            if (within_tolerance || overshot) {
                return ActionStatus::SUCCESS;
            }
        }

        if (elapsed_ticks() > 50) {
            if (avg_velocity < 0.3) {
                ++stall_counter_;
                if (stall_counter_ >= 200) {
                    return ActionStatus::SUCCESS;
                }
            } else {
                stall_counter_ = 0;
            }
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override { belt_torque_offset_ = 0.0; }

private:
    rmcs_msgs::DartSliderStatus& belt_command_;
    double& belt_target_velocity_;
    double& belt_torque_offset_;
    double& belt_torque_limit_;
    double torque_offset_value_;
    const double& left_belt_angle_;
    const double& right_belt_angle_;
    const double& left_belt_velocity_;
    const double& right_belt_velocity_;

    double target_distance_;      // 目标距离（m）
    double velocity_;             // 运动速度（rad/s）
    double torque_limit_;         // 扭矩限制（N⋅m）
    uint64_t timeout_ticks_;
    uint64_t min_running_ticks_;

    double initial_angle_{0.0};   // 初始角度（rad，在on_enter中读取）
    double target_position_{0.0}; // 实际目标位置（rad，在on_enter中计算）
    uint64_t stall_counter_{0};
};

} // namespace rmcs_dart_guidance::manager
