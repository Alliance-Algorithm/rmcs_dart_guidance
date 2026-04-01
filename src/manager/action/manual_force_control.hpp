#pragma once

#include "action.hpp"

#include <eigen3/Eigen/Dense>

namespace rmcs_dart_guidance::manager {

// 手动力控：右摇杆控制拉力电机速度。
// 任务将持续运行，直到被外部模式切换指令抢占取消。
class DartManualForceControlAction : public IAction {
public:
    DartManualForceControlAction(
        double& force_control_velocity, const Eigen::Vector2d& joystick_right,
        double max_transform_rate, double manual_force_scale = 5.0)
        : IAction("dart_manual_force_control")
        , force_control_velocity_(force_control_velocity)
        , joystick_right_(joystick_right)
        , max_transform_rate_(max_transform_rate)
        , manual_force_scale_(manual_force_scale) {}

    void on_enter() override {}

    ActionStatus update() override {
        force_control_velocity_ = joystick_right_.x() * max_transform_rate_ * manual_force_scale_;

        return ActionStatus::RUNNING;
    }

private:
    double& force_control_velocity_;
    const Eigen::Vector2d& joystick_right_;

    double max_transform_rate_;
    double manual_force_scale_;
};

} // namespace rmcs_dart_guidance::manager
