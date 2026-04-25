#pragma once

#include "action.hpp"
#include <cmath>
#include <cstdint>

namespace rmcs_dart_guidance::manager {

// ForceScrewCalibrationAction — 力矩闭环控制动作
//   通过PID控制force_screw_motor，使当前力闭环到目标力值
//   电机正转力增大，反转力减小
//   支持两种力值输入源：
//     1. 原始传感器 (current_force_ch1/ch2)
//     2. 卡尔曼滤波后 (filtered_force, 由 ForceKalmanFilter 组件提供)
class ForceScrewCalibrationAction : public IAction {
public:
    // Constructor for raw sensor input (legacy mode)
    // force_channel: 1 = ch1, 2 = ch2
    ForceScrewCalibrationAction(
        std::string name, double& force_screw_velocity, const int& current_force_ch1,
        const int& current_force_ch2, int force_channel, double target_force,
        double force_tolerance, uint64_t settle_ticks, uint64_t timeout_ticks, double kp, double ki,
        double kd, double max_velocity)
        : IAction(std::move(name))
        , force_screw_velocity_(force_screw_velocity)
        , current_force_ch1_(current_force_ch1)
        , current_force_ch2_(current_force_ch2)
        , filtered_force_(nullptr)
        , force_rate_(nullptr)
        , force_channel_(force_channel)
        , target_force_(target_force)
        , force_tolerance_(force_tolerance)
        , settle_ticks_(settle_ticks)
        , timeout_ticks_(timeout_ticks)
        , kp_(kp)
        , ki_(ki)
        , kd_(kd)
        , max_velocity_(max_velocity)
        , use_filtered_input_(false)
        , use_rate_feedforward_(false)
        , rate_gain_(0.0) {}

    // Constructor for Kalman-filtered input (new mode)
    ForceScrewCalibrationAction(
        std::string name, double& force_screw_velocity, const double& filtered_force,
        const double& force_rate, double target_force, double force_tolerance,
        uint64_t settle_ticks, uint64_t timeout_ticks, double kp, double ki, double kd,
        double max_velocity, bool use_rate_feedforward = false, double rate_gain = 0.0)
        : IAction(std::move(name))
        , force_screw_velocity_(force_screw_velocity)
        , current_force_ch1_(*(int*)nullptr) // dummy, won't be used
        , current_force_ch2_(*(int*)nullptr)
        , filtered_force_(&filtered_force)
        , force_rate_(&force_rate)
        , force_channel_(0)
        , target_force_(target_force)
        , force_tolerance_(force_tolerance)
        , settle_ticks_(settle_ticks)
        , timeout_ticks_(timeout_ticks)
        , kp_(kp)
        , ki_(ki)
        , kd_(kd)
        , max_velocity_(max_velocity)
        , use_filtered_input_(true)
        , use_rate_feedforward_(use_rate_feedforward)
        , rate_gain_(rate_gain) {}

    void on_enter() override {
        integral_ = 0.0;
        last_error_ = 0.0;
        settle_counter_ = 0;
    }

    ActionStatus update() override {
        if (elapsed_ticks() >= timeout_ticks_) {
            return ActionStatus::SUCCESS;
        }

        // Get current force from either raw sensors or filtered input
        double current_force;
        if (use_filtered_input_) {
            // Kalman-filtered force (already in Newtons)
            current_force = *filtered_force_;
        } else {
            // Raw sensor (in grams, convert to Newtons)
            current_force =
                static_cast<double>(force_channel_ == 2 ? current_force_ch2_ : current_force_ch1_) *
                0.00981;
        }

        double error = (target_force_ - current_force);
        if (!use_filtered_input_) {
            error *= 5; // Legacy scaling for raw sensors
        }

        integral_ += error;
        double derivative = error - last_error_;
        last_error_ = error;

        // PID output
        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

        // Optional rate feedforward (only for filtered input)
        if (use_filtered_input_ && use_rate_feedforward_ && force_rate_ != nullptr) {
            output -= rate_gain_ * (*force_rate_);
        }

        // Clamp output
        output = std::clamp(output, -max_velocity_, max_velocity_);

        // Output to motor
        force_screw_velocity_ = output;

        // Check if within tolerance
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

    void on_exit() override {
        force_screw_velocity_ = 0.0;
        integral_ = 0.0;
    }

private:
    double& force_screw_velocity_;
    const int& current_force_ch1_;
    const int& current_force_ch2_;
    const double* filtered_force_;  // nullptr if using raw sensors
    const double* force_rate_;      // nullptr if using raw sensors
    int force_channel_;             // 1 = ch1, 2 = ch2 (only for raw mode)
    double target_force_;
    double force_tolerance_;
    uint64_t settle_ticks_;
    uint64_t timeout_ticks_;
    double kp_;
    double ki_;
    double kd_;
    double max_velocity_;
    bool use_filtered_input_;       // true = use filtered_force_, false = use raw sensors
    bool use_rate_feedforward_;     // true = use dF/dt feedforward (filtered mode only)
    double rate_gain_;              // feedforward gain
    double integral_{0.0};
    double last_error_{0.0};
    uint64_t settle_counter_{0};
};

} // namespace rmcs_dart_guidance::manager
