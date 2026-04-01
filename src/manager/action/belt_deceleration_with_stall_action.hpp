#pragma once

#include "action.hpp"
#include <cmath>
#include <cstdint>

namespace rmcs_dart_guidance::manager {

// BeltDecelerationWithStallAction - 减速并监测堵转
//   用于上行最后阶段：减速到目标速度，监测堵转作为成功标志
//   与 BeltPIDDecelerationAction 的区别：
//   - 目标速度可配置（不一定是0）
//   - 堵转检测返回 SUCCESS（而不是 FAILURE）
class BeltDecelerationWithStallAction : public IAction {
public:
    BeltDecelerationWithStallAction(
        std::string name, double& belt_target_velocity, double& belt_torque_offset,
        const double& left_belt_velocity, const double& right_belt_velocity,
        const double& left_belt_torque, const double& right_belt_torque,
        double target_velocity,          // 目标速度（rad/s）
        double torque_offset_value,      // 力矩偏移（N⋅m）
        double stall_velocity_threshold, // 堵转速度阈值（rad/s）
        double stall_torque_threshold,   // 堵转扭矩阈值（N⋅m）
        uint64_t stall_confirm_ticks,    // 堵转确认帧数
        uint64_t min_running_ticks,      // 最小运行帧数
        uint64_t timeout_ticks           // 超时帧数
        )
        : IAction(std::move(name))
        , belt_target_velocity_(belt_target_velocity)
        , belt_torque_offset_(belt_torque_offset)
        , left_belt_velocity_(left_belt_velocity)
        , right_belt_velocity_(right_belt_velocity)
        , left_belt_torque_(left_belt_torque)
        , right_belt_torque_(right_belt_torque)
        , target_velocity_(target_velocity)
        , torque_offset_value_(torque_offset_value)
        , stall_velocity_threshold_(stall_velocity_threshold)
        , stall_torque_threshold_(stall_torque_threshold)
        , stall_confirm_ticks_(stall_confirm_ticks)
        , min_running_ticks_(min_running_ticks)
        , timeout_ticks_(timeout_ticks) {}

    void on_enter() override {
        belt_target_velocity_ = target_velocity_;
        belt_torque_offset_ = torque_offset_value_;
        stall_counter_ = 0;
        printf(
            "[%s] on_enter: target_velocity=%.4f, torque_offset=%.4f\n", name().c_str(),
            target_velocity_, torque_offset_value_);
    }

    ActionStatus update() override {
        // 超时检测
        if (elapsed_ticks() >= timeout_ticks_) {
            return ActionStatus::FAILURE;
        }

        // 计算平均速度和扭矩
        double avg_velocity =
            (std::abs(left_belt_velocity_) + std::abs(right_belt_velocity_)) / 2.0;
        bool torque_active = std::abs(left_belt_torque_) > stall_torque_threshold_
                          || std::abs(right_belt_torque_) > stall_torque_threshold_;

        // 堵转检测（最小运行帧数后才开始检测）：速度低且扭矩高
        if (elapsed_ticks() > min_running_ticks_) {
            if (avg_velocity < stall_velocity_threshold_ && torque_active) {
                ++stall_counter_;
                if (stall_counter_ >= stall_confirm_ticks_) {
                    return ActionStatus::SUCCESS;
                }
            } else {
                stall_counter_ = 0;
            }
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override { belt_torque_offset_ = 0.0; }

private:
    double& belt_target_velocity_;
    double& belt_torque_offset_;
    const double& left_belt_velocity_;
    const double& right_belt_velocity_;
    const double& left_belt_torque_;
    const double& right_belt_torque_;
    double target_velocity_;
    double torque_offset_value_;
    double stall_velocity_threshold_;
    double stall_torque_threshold_;
    uint64_t stall_confirm_ticks_;
    uint64_t min_running_ticks_;
    uint64_t timeout_ticks_;
    uint64_t stall_counter_{0};
};

} // namespace rmcs_dart_guidance::manager
