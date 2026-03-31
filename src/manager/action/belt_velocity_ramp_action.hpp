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
        std::string name, rmcs_msgs::DartSliderStatus& belt_command, double& belt_target_velocity,
        double& belt_torque_limit, double& belt_hold_torque, bool& belt_wait_zero_velocity,
        const double& left_belt_velocity, const double& right_belt_velocity,
        uint64_t target_ramp_ticks, double torque_limit, double hold_torque,
        double zero_velocity_threshold, uint64_t zero_confirm_ticks, uint64_t timeout_ticks,
        double initial_velocity)
        : IAction(std::move(name))
        , belt_command_(belt_command)
        , belt_target_velocity_(belt_target_velocity)
        , belt_torque_limit_(belt_torque_limit)
        , belt_hold_torque_(belt_hold_torque)
        , belt_wait_zero_velocity_(belt_wait_zero_velocity)
        , left_belt_velocity_(left_belt_velocity)
        , right_belt_velocity_(right_belt_velocity)
        , target_ramp_ticks_(target_ramp_ticks)
        , torque_limit_(std::max(0.0, std::abs(torque_limit)))
        , hold_torque_(hold_torque)
        , zero_velocity_threshold_(std::max(0.0, std::abs(zero_velocity_threshold)))
        , zero_confirm_ticks_(zero_confirm_ticks)
        , timeout_ticks_(timeout_ticks)
        , initial_velocity_(std::abs(initial_velocity)) {}

    void on_enter() override {
        belt_torque_limit_ = torque_limit_;
        belt_hold_torque_ = hold_torque_;

        // 使用传入的初始速度，而不是读取可能错误的速度反馈
        // 但如果上一个action已经设置了belt_target_velocity_，优先使用它以保证连续性
        current_velocity_ =
            (belt_target_velocity_ > 1e-6) ? belt_target_velocity_ : initial_velocity_;
        belt_target_velocity_ = current_velocity_;

        // 基于初始速度和目标减速时长，动态计算减速步长
        // 这样无论初始速度是多少，减速时间都是 target_ramp_ticks
        ramp_step_per_tick_ = target_ramp_ticks_ > 0
                                ? (current_velocity_ / static_cast<double>(target_ramp_ticks_))
                                : current_velocity_;

        zero_counter_ = 0;

        // 调试日志：打印关键参数
        static int enter_count = 0;
        ++enter_count;
        printf(
            "[BeltModerateAction::on_enter #%d] initial_velocity_=%.4f, current_velocity_=%.4f, "
            "belt_target_velocity_=%.4f, ramp_step=%.6f\n",
            enter_count, initial_velocity_, current_velocity_, belt_target_velocity_,
            ramp_step_per_tick_);
    }

    ActionStatus update() override {
        if (elapsed_ticks() >= timeout_ticks_) {
            return ActionStatus::FAILURE;
        }

        current_velocity_ = std::max(0.0, current_velocity_ - ramp_step_per_tick_);
        belt_target_velocity_ = current_velocity_;

        if (current_velocity_ <= 1e-6) {
            const double avg_vel =
                (std::abs(left_belt_velocity_) + std::abs(right_belt_velocity_)) / 2.0;
            if (avg_vel < zero_velocity_threshold_) {
                ++zero_counter_;
                if (zero_counter_ >= zero_confirm_ticks_) {
                    return ActionStatus::SUCCESS;
                }
            } else {
                zero_counter_ = 0;
            }
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        // 退出时设置为WAIT模式，但保持扭矩和参数设置
        // 这样下一个action可以无缝接管
        belt_command_ = rmcs_msgs::DartSliderStatus::WAIT;
        belt_target_velocity_ = 0.0;
        // belt_torque_limit_和belt_hold_torque_保持不变，由下一个action接管
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

    uint64_t target_ramp_ticks_;
    double ramp_step_per_tick_{0.0};
    double torque_limit_;
    double hold_torque_;
    double zero_velocity_threshold_;
    uint64_t zero_confirm_ticks_;
    uint64_t timeout_ticks_;
    double initial_velocity_; // 减速起点速度

    double current_velocity_{0.0};
    uint64_t zero_counter_{0};
};

} // namespace rmcs_dart_guidance::manager
