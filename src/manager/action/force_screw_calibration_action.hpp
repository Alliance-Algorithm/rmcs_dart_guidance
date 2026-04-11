#pragma once

#include "action.hpp"
#include <cmath>
#include <cstdint>

namespace rmcs_dart_guidance::manager {

// ForceScrewCalibrationAction — 力矩闭环控制动作
//   通过PID控制force_screw_motor，使当前力闭环到目标力值
//   电机正转力增大，反转力减小
class ForceScrewCalibrationAction : public IAction {
public:
    // force_channel: 1 = ch1, 2 = ch2
    ForceScrewCalibrationAction(
        std::string name, double& force_screw_velocity, const int& current_force_ch1,
        const int& current_force_ch2, int force_channel, double target_force,
        double force_tolerance, uint64_t settle_ticks, uint64_t timeout_ticks, double kp,
        double ki, double kd, double max_velocity)
        : IAction(std::move(name))
        , force_screw_velocity_(force_screw_velocity)
        , current_force_ch1_(current_force_ch1)
        , current_force_ch2_(current_force_ch2)
        , force_channel_(force_channel)
        , target_force_(target_force)
        , force_tolerance_(force_tolerance)
        , settle_ticks_(settle_ticks)
        , timeout_ticks_(timeout_ticks)
        , kp_(kp)
        , ki_(ki)
        , kd_(kd)
        , max_velocity_(max_velocity) {}

    void on_enter() override {
        integral_ = 0.0;
        last_error_ = 0.0;
        settle_counter_ = 0;
    }

    ActionStatus update() override {
        if (elapsed_ticks() >= timeout_ticks_) {
            return ActionStatus::SUCCESS;
        }

        double current_force =
            static_cast<double>(force_channel_ == 2 ? current_force_ch2_ : current_force_ch1_);
        double error = (target_force_ - current_force) * 5; // 增加响应速度

        integral_ += error;
        double derivative = error - last_error_;
        last_error_ = error;

        // PID输出
        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

        // 限幅
        if (output > max_velocity_) {
            output = max_velocity_;
        } else if (output < -max_velocity_) {
            output = -max_velocity_;
        }

        // 输出到电机
        force_screw_velocity_ = output;

        // 检查是否在容差范围内
        if (std::abs(error) <= force_tolerance_) {
            ++settle_counter_;
            if (settle_counter_ >= settle_ticks_) {
                return ActionStatus::SUCCESS;
            }
        } else {
            settle_counter_ = 0;
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override { force_screw_velocity_ = 0.0; }

private:
    double& force_screw_velocity_;
    const int& current_force_ch1_;
    const int& current_force_ch2_;
    int force_channel_; // 1 = ch1, 2 = ch2
    double target_force_;
    double force_tolerance_;
    uint64_t settle_ticks_;
    uint64_t timeout_ticks_;
    double kp_;
    double ki_;
    double kd_;
    double max_velocity_;
    double integral_{0.0};
    double last_error_{0.0};
    uint64_t settle_counter_{0};
};

} // namespace rmcs_dart_guidance::manager
