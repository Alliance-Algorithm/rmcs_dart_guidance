#pragma once

#include "action.hpp"

#include <cmath>
#include <cstdint>

#include <eigen3/Eigen/Dense>

namespace rmcs_dart_guidance::manager {

// 手动力控：右摇杆控制拉力电机速度。
// 默认持续运行；若检测到丝杆堵转，则返回 SUCCESS 并停车。
class DartManualForceControlAction : public IAction {
public:
    DartManualForceControlAction(
        double& force_control_velocity, const Eigen::Vector2d& joystick_right,
        double max_transform_rate, double manual_force_scale = 5.0,
        const double* force_screw_velocity_feedback = nullptr,
        const double* force_screw_torque_feedback = nullptr)
        : IAction("dart_manual_force_control")
        , force_control_velocity_(force_control_velocity)
        , joystick_right_(joystick_right)
        , max_transform_rate_(max_transform_rate)
        , manual_force_scale_(manual_force_scale)
        , force_screw_velocity_feedback_(force_screw_velocity_feedback)
        , force_screw_torque_feedback_(force_screw_torque_feedback) {}

    void on_enter() override {
        force_control_velocity_ = 0.0;
        stall_counter_ = 0;
        if (manual_force_scale_ > max_transform_rate_) {
            manual_force_scale_ = max_transform_rate_;
        }
    }

    ActionStatus update() override {
        const double target_velocity = joystick_right_.x() * manual_force_scale_;
        force_control_velocity_ = target_velocity;

        if (std::abs(target_velocity) <= 1e-6 || force_screw_velocity_feedback_ == nullptr
            || force_screw_torque_feedback_ == nullptr) {
            stall_counter_ = 0;
            return ActionStatus::RUNNING;
        }

        if (elapsed_ticks() > 100) {
            const double measured_velocity = std::abs(*force_screw_velocity_feedback_);
            const double measured_torque = std::abs(*force_screw_torque_feedback_);

            if (measured_velocity < 0.15 && measured_torque > 0.5) {
                ++stall_counter_;
                if (stall_counter_ >= 100) {
                    return ActionStatus::SUCCESS;
                }
            } else {
                stall_counter_ = 0;
            }
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        force_control_velocity_ = 0.0;
        stall_counter_ = 0;
    }

private:
    double& force_control_velocity_;
    const Eigen::Vector2d& joystick_right_;

    double max_transform_rate_;
    double manual_force_scale_;
    const double* force_screw_velocity_feedback_;
    const double* force_screw_torque_feedback_;
    uint64_t stall_counter_{0};
};

} // namespace rmcs_dart_guidance::manager
