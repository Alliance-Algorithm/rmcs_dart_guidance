#pragma once

#include "action.hpp"
#include <cmath>
#include <cstdint>

namespace rmcs_dart_guidance::manager {

class BeltPIDDecelerationAction : public IAction {
public:
    BeltPIDDecelerationAction(
        std::string name, double& belt_target_velocity, double& belt_torque_offset,
        const double& left_belt_velocity, const double& right_belt_velocity,
        double torque_offset_value, double zero_velocity_threshold, uint64_t zero_confirm_ticks,
        uint64_t timeout_ticks)
        : IAction(std::move(name))
        , belt_target_velocity_(belt_target_velocity)
        , belt_torque_offset_(belt_torque_offset)
        , left_belt_velocity_(left_belt_velocity)
        , right_belt_velocity_(right_belt_velocity)
        , torque_offset_value_(torque_offset_value)
        , zero_velocity_threshold_(zero_velocity_threshold)
        , zero_confirm_ticks_(zero_confirm_ticks)
        , timeout_ticks_(timeout_ticks) {}

    void on_enter() override {
        belt_target_velocity_ = 0.0;
        belt_torque_offset_ = torque_offset_value_;
        zero_counter_ = 0;
    }

    ActionStatus update() override {
        if (elapsed_ticks() >= timeout_ticks_) {
            return ActionStatus::SUCCESS;
        }

        double avg_velocity =
            (std::abs(left_belt_velocity_) + std::abs(right_belt_velocity_)) / 2.0;

        if (avg_velocity < zero_velocity_threshold_) {
            ++zero_counter_;
            if (zero_counter_ >= zero_confirm_ticks_) {
                return ActionStatus::SUCCESS;
            }
        } else {
            zero_counter_ = 0;
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override { belt_torque_offset_ = 0.0; }

private:
    double& belt_target_velocity_;
    double& belt_torque_offset_;
    const double& left_belt_velocity_;
    const double& right_belt_velocity_;
    double torque_offset_value_;
    double zero_velocity_threshold_;
    uint64_t zero_confirm_ticks_;
    uint64_t timeout_ticks_;
    uint64_t zero_counter_{0};
};

} // namespace rmcs_dart_guidance::manager
