#pragma once

#include "action.hpp"

#include <cmath>
#include <cstdint>
#include <string>

#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

// BeltUpInitialAccelAction - 上行初始加速阶段
//   前0.1m慢速（5 rad/s）+ 常态力矩偏移补偿负载
//   完成条件：行进距离达到0.2m
class BeltUpInitialAccelAction : public IAction {
public:
    BeltUpInitialAccelAction(
        std::string name, rmcs_msgs::DartSliderStatus& belt_command, double& belt_target_velocity,
        double& belt_torque_offset, double& belt_torque_limit, double& belt_error_gain,
        bool& belt_use_decel_pid, const double& left_belt_angle, const double& right_belt_angle,
        double pulley_radius, double slow_velocity, const double& torque_offset_value, double torque_limit,
        double accel_distance = 0.1, uint64_t timeout_ticks = 5000)
        : IAction(std::move(name))
        , belt_command_(belt_command)
        , belt_target_velocity_(belt_target_velocity)
        , belt_torque_offset_(belt_torque_offset)
        , belt_torque_limit_(belt_torque_limit)
        , belt_error_gain_(belt_error_gain)
        , belt_use_decel_pid_(belt_use_decel_pid)
        , left_belt_angle_(left_belt_angle)
        , right_belt_angle_(right_belt_angle)
        , pulley_radius_(pulley_radius)
        , slow_velocity_(slow_velocity)
        , torque_offset_value_(torque_offset_value)
        , torque_limit_(torque_limit)
        , accel_distance_(accel_distance)
        , timeout_ticks_(timeout_ticks) {}

    void on_enter() override {
        initial_angle_ = (left_belt_angle_ + right_belt_angle_) / 2.0;
        belt_command_ = rmcs_msgs::DartSliderStatus::UP;
        belt_target_velocity_ = slow_velocity_;
        belt_torque_offset_ = torque_offset_value_;
        belt_torque_limit_ = torque_limit_;
        belt_error_gain_ = 1.0;          // 恢复正常error增益
        belt_use_decel_pid_ = false;     // 恢复正常PID参数
    }

    ActionStatus update() override {
        if (elapsed_ticks() >= timeout_ticks_) {
            return ActionStatus::SUCCESS;
        }

        double avg_angle = (left_belt_angle_ + right_belt_angle_) / 2.0;
        double distance_traveled = std::abs(avg_angle - initial_angle_) * pulley_radius_;

        if (distance_traveled >= accel_distance_) {
            return ActionStatus::SUCCESS;
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override { belt_torque_offset_ = 0.0; }

private:
    rmcs_msgs::DartSliderStatus& belt_command_;
    double& belt_target_velocity_;
    double& belt_torque_offset_;
    double& belt_torque_limit_;
    double& belt_error_gain_;
    bool& belt_use_decel_pid_;
    const double& left_belt_angle_;
    const double& right_belt_angle_;

    double pulley_radius_;
    double slow_velocity_;
    const double& torque_offset_value_;
    double torque_limit_;
    double accel_distance_;
    uint64_t timeout_ticks_;

    double initial_angle_{0.0};
};

} // namespace rmcs_dart_guidance::manager
