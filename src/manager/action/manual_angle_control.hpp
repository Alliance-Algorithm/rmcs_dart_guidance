#pragma once

#include "action.hpp"

#include <eigen3/Eigen/Dense>

namespace rmcs_dart_guidance::manager {

// 手动角度控制：左摇杆控制 yaw，右摇杆控制 pitch。
// 任务将持续运行，直到被外部模式切换指令抢占取消。
class DartManualAngleControlAction : public IAction {
public:
    DartManualAngleControlAction(
        double& yaw_control_velocity, double& pitch_control_velocity,
        const Eigen::Vector2d& joystick_left, const Eigen::Vector2d& joystick_right,
        double max_transform_rate)
        : IAction("dart_manual_angle_control")
        , yaw_control_velocity_(yaw_control_velocity)
        , pitch_control_velocity_(pitch_control_velocity)
        , joystick_left_(joystick_left)
        , joystick_right_(joystick_right)
        , max_transform_rate_(max_transform_rate) {}

    void on_enter() override {}

    ActionStatus update() override {
        yaw_control_velocity_ = joystick_left_.y() * max_transform_rate_;
        pitch_control_velocity_ = joystick_right_.x() * max_transform_rate_;

        return ActionStatus::RUNNING;
    }

private:
    double& yaw_control_velocity_;
    double& pitch_control_velocity_;

    const Eigen::Vector2d& joystick_left_;
    const Eigen::Vector2d& joystick_right_;

    double max_transform_rate_;
};

} // namespace rmcs_dart_guidance::manager
