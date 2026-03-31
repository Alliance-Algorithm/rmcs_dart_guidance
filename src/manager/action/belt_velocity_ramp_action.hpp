#pragma once

#include "action.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

class BeltModerateAction : public IAction {
public:
    BeltModerateAction(
        std::string name, rmcs_msgs::DartSliderStatus& belt_command,
        double& belt_target_velocity, double& belt_torque_limit, double& belt_hold_torque,
        bool& belt_wait_zero_velocity, const double& left_belt_velocity,
        const double& right_belt_velocity, rmcs_msgs::DartSliderStatus move_command,
        double start_velocity, double ramp_step_per_tick, double torque_limit, double hold_torque,
        double zero_velocity_threshold, uint64_t zero_confirm_ticks, uint64_t timeout_ticks)
        : IAction(std::move(name))
        , belt_command_(belt_command)
        , belt_target_velocity_(belt_target_velocity)
        , belt_torque_limit_(belt_torque_limit)
        , belt_hold_torque_(belt_hold_torque)
        , belt_wait_zero_velocity_(belt_wait_zero_velocity)
        , left_belt_velocity_(left_belt_velocity)
        , right_belt_velocity_(right_belt_velocity)
        , move_command_(move_command)
        , start_velocity_(std::max(0.0, std::abs(start_velocity)))
        , ramp_step_per_tick_(std::max(0.0, std::abs(ramp_step_per_tick)))
        , torque_limit_(std::max(0.0, std::abs(torque_limit)))
        , hold_torque_(hold_torque)
        , zero_velocity_threshold_(std::max(0.0, std::abs(zero_velocity_threshold)))
        , zero_confirm_ticks_(zero_confirm_ticks)
        , timeout_ticks_(timeout_ticks) {}

    void on_enter() override {
        // 保持当前运动方向，不改变 belt_command_（避免方向突变）
        belt_torque_limit_ = torque_limit_;
        belt_hold_torque_ = hold_torque_;

        // 读取当前实际速度作为起始速度（避免速度突变）
        const double avg_vel =
            (std::abs(left_belt_velocity_) + std::abs(right_belt_velocity_)) / 2.0;

        // 使用实际速度和设定速度中的较大值，确保平滑过渡
        current_velocity_ = std::max(avg_vel, start_velocity_);
        belt_target_velocity_ = current_velocity_;

        zero_counter_ = 0;

        printf("[%s] on_enter: start_velocity=%.4f, actual_velocity=%.4f, using=%.4f\n",
               name().c_str(), start_velocity_, avg_vel, current_velocity_);
    }

    ActionStatus update() override {
        if (elapsed_ticks() >= timeout_ticks_) {
            printf("[%s] TIMEOUT after %lu ticks\n", name().c_str(), elapsed_ticks());
            return ActionStatus::FAILURE;
        }

        // 平滑减速：每个 tick 减少固定步长
        current_velocity_ = std::max(0.0, current_velocity_ - ramp_step_per_tick_);
        belt_target_velocity_ = current_velocity_;

        // 每100帧打印一次调试信息
        if (elapsed_ticks() % 100 == 0) {
            const double avg_vel =
                (std::abs(left_belt_velocity_) + std::abs(right_belt_velocity_)) / 2.0;
            printf("[%s] tick=%lu, target_vel=%.4f, actual_vel=%.4f\n",
                   name().c_str(), elapsed_ticks(), current_velocity_, avg_vel);
        }

        // 当目标速度接近0时，检查实际速度是否也接近0
        if (current_velocity_ <= 1e-6) {
            const double avg_vel =
                (std::abs(left_belt_velocity_) + std::abs(right_belt_velocity_)) / 2.0;
            if (avg_vel < zero_velocity_threshold_) {
                ++zero_counter_;
                if (zero_counter_ >= zero_confirm_ticks_) {
                    printf("[%s] SUCCESS: velocity reached zero (actual_vel=%.4f < threshold=%.4f)\n",
                           name().c_str(), avg_vel, zero_velocity_threshold_);
                    return ActionStatus::SUCCESS;
                }
            } else {
                zero_counter_ = 0;
            }
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        // 减速完成，进入 WAIT 模式
        belt_command_ = rmcs_msgs::DartSliderStatus::WAIT;
        belt_target_velocity_ = 0.0;  // 目标速度为 0
        belt_torque_limit_ = torque_limit_;
        belt_hold_torque_ = hold_torque_;
        belt_wait_zero_velocity_ = false;
    }

private:
    rmcs_msgs::DartSliderStatus& belt_command_;
    double& belt_target_velocity_;
    double& belt_torque_limit_;
    double& belt_hold_torque_;
    bool& belt_wait_zero_velocity_;
    const double& left_belt_velocity_;
    const double& right_belt_velocity_;

    rmcs_msgs::DartSliderStatus move_command_;
    double start_velocity_;
    double ramp_step_per_tick_;
    double torque_limit_;
    double hold_torque_;
    double zero_velocity_threshold_;
    uint64_t zero_confirm_ticks_;
    uint64_t timeout_ticks_;

    double current_velocity_{0.0};
    uint64_t zero_counter_{0};
};

} // namespace rmcs_dart_guidance::manager
