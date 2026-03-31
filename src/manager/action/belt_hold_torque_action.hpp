#pragma once

#include "action.hpp"

#include <cstdint>
#include <string>

#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

// BeltHoldTorqueAction - 保持传送带张力
//   持续输出 WAIT 命令 + hold_torque，用于在扳机锁定期间保持传送带张力
class BeltHoldTorqueAction : public IAction {
public:
    BeltHoldTorqueAction(
        std::string name, rmcs_msgs::DartSliderStatus& belt_command, double& belt_target_velocity,
        double& belt_hold_torque, bool& belt_wait_zero_velocity, double hold_torque_value,
        uint64_t hold_duration_ticks)
        : IAction(std::move(name))
        , belt_command_(belt_command)
        , belt_target_velocity_(belt_target_velocity)
        , belt_hold_torque_(belt_hold_torque)
        , belt_wait_zero_velocity_(belt_wait_zero_velocity)
        , hold_torque_value_(hold_torque_value)
        , hold_duration_ticks_(hold_duration_ticks) {}

    void on_enter() override {
        // 立即设置WAIT命令和保持扭矩，确保无缝过渡
        belt_command_ = rmcs_msgs::DartSliderStatus::WAIT;
        belt_target_velocity_ = 0.0;
        belt_hold_torque_ = hold_torque_value_;
        belt_wait_zero_velocity_ = false; // 使用 HOLD_TORQUE 模式
    }

    ActionStatus update() override {
        if (elapsed_ticks() >= hold_duration_ticks_) {
            return ActionStatus::SUCCESS;
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override {}

private:
    rmcs_msgs::DartSliderStatus& belt_command_;
    double& belt_target_velocity_;
    double& belt_hold_torque_;
    bool& belt_wait_zero_velocity_;
    double hold_torque_value_;
    uint64_t hold_duration_ticks_;
};

} // namespace rmcs_dart_guidance::manager
