#pragma once

#include "action.hpp"

#include <cstdint>
#include <string>

#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

// BeltHoldTorqueAction - 保持传送带张力
//   使用零速度闭环 + 常态扭矩偏移，用于在扳机锁定期间保持传送带张力
//   与减速阶段一致，使用PID控制速度为0，同时叠加常态扭矩偏移补偿负载
class BeltHoldTorqueAction : public IAction {
public:
    BeltHoldTorqueAction(
        std::string name, rmcs_msgs::DartSliderStatus& belt_command, double& belt_target_velocity,
        double& belt_hold_torque, bool& belt_wait_zero_velocity, double& belt_torque_offset,
        double hold_torque_value, const double& torque_offset_value, uint64_t hold_duration_ticks)
        : IAction(std::move(name))
        , belt_command_(belt_command)
        , belt_target_velocity_(belt_target_velocity)
        , belt_hold_torque_(belt_hold_torque)
        , belt_wait_zero_velocity_(belt_wait_zero_velocity)
        , belt_torque_offset_(belt_torque_offset)
        , hold_torque_value_(hold_torque_value)
        , torque_offset_value_(torque_offset_value)
        , hold_duration_ticks_(hold_duration_ticks) {}

    void on_enter() override {
        belt_command_ = rmcs_msgs::DartSliderStatus::WAIT;
        belt_target_velocity_ = 0.0;
        belt_hold_torque_ = hold_torque_value_;
        belt_wait_zero_velocity_ = true;            // 使用 HOLD_TORQUE 模式（常数力矩）
        belt_torque_offset_ = torque_offset_value_; // 叠加常态扭矩偏移补偿负载
    }

    ActionStatus update() override {
        if (elapsed_ticks() >= hold_duration_ticks_) {
            return ActionStatus::SUCCESS;
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        belt_hold_torque_ = 0.0;
        belt_torque_offset_ = 0.0;
    }

private:
    rmcs_msgs::DartSliderStatus& belt_command_;
    double& belt_target_velocity_;
    double& belt_hold_torque_;
    bool& belt_wait_zero_velocity_;
    double& belt_torque_offset_;
    double hold_torque_value_;
    const double& torque_offset_value_;
    uint64_t hold_duration_ticks_;
};

} // namespace rmcs_dart_guidance::manager
