#pragma once

#include "action.hpp"

#include <functional>
#include <rmcs_msgs/dart_slider_status.hpp>

#include <cmath>
#include <cstdint>

namespace rmcs_dart_guidance::manager {

// BeltDecelerationAction - 传送带减速动作
//   支持多种减速模式：
//   1. 零速检测模式：减速到零速，检测速度低于阈值作为成功条件
//   2. 堵转检测模式：减速到目标速度，检测堵转（低速+高扭矩）作为成功条件
//   3. 混合模式：同时支持零速和堵转检测
//
//   参数说明：
//   - target_velocity: 目标速度（rad/s），通常为0或小的正值
//   - torque_offset_value: 力矩偏移（N⋅m），用于补偿负载
//   - enable_stall_detection: 是否启用堵转检测
//   - enable_zero_velocity_detection: 是否启用零速检测
//   - stall_velocity_threshold: 堵转速度阈值（rad/s）
//   - stall_torque_threshold: 堵转扭矩阈值（N⋅m）
//   - zero_velocity_threshold: 零速阈值（rad/s）
//   - stall_confirm_ticks: 堵转确认帧数
//   - zero_confirm_ticks: 零速确认帧数
//   - min_running_ticks: 最小运行帧数（避免启动瞬间误触发）
//   - timeout_ticks: 超时帧数
class BeltDecelerationAction : public IAction {
public:
    BeltDecelerationAction(
        std::string name, double& belt_target_velocity, double& belt_torque_offset,
        double& belt_error_gain, bool& belt_use_decel_pid,
        const double& left_belt_velocity, const double& right_belt_velocity,
        const double& left_belt_torque, const double& right_belt_torque,
        double target_velocity = 0.0, const double& torque_offset_value = kZero_,
        double error_gain_value = 2.0,  // 默认2倍error增益
        bool enable_stall_detection = false, bool enable_zero_velocity_detection = true,
        double stall_velocity_threshold = 0.5, double stall_torque_threshold = 0.5,
        double zero_velocity_threshold = 0.1, uint64_t stall_confirm_ticks = 100,
        uint64_t zero_confirm_ticks = 100, uint64_t min_running_ticks = 50,
        uint64_t timeout_ticks = 3000,
        rmcs_msgs::DartSliderStatus* belt_command = nullptr,
        bool* belt_wait_zero_velocity = nullptr, bool send_wait_zero_on_exit = false,
        std::function<void()> on_enter_hook = nullptr)
        : IAction(std::move(name))
        , belt_target_velocity_(belt_target_velocity)
        , belt_torque_offset_(belt_torque_offset)
        , belt_error_gain_(belt_error_gain)
        , belt_use_decel_pid_(belt_use_decel_pid)
        , left_belt_velocity_(left_belt_velocity)
        , right_belt_velocity_(right_belt_velocity)
        , left_belt_torque_(left_belt_torque)
        , right_belt_torque_(right_belt_torque)
        , target_velocity_(target_velocity)
        , torque_offset_value_(torque_offset_value)
        , error_gain_value_(error_gain_value)
        , enable_stall_detection_(enable_stall_detection)
        , enable_zero_velocity_detection_(enable_zero_velocity_detection)
        , stall_velocity_threshold_(stall_velocity_threshold)
        , stall_torque_threshold_(stall_torque_threshold)
        , zero_velocity_threshold_(zero_velocity_threshold)
        , stall_confirm_ticks_(stall_confirm_ticks)
        , zero_confirm_ticks_(zero_confirm_ticks)
        , min_running_ticks_(min_running_ticks)
        , timeout_ticks_(timeout_ticks)
        , belt_command_(belt_command)
        , belt_wait_zero_velocity_(belt_wait_zero_velocity)
        , send_wait_zero_on_exit_(send_wait_zero_on_exit)
        , on_enter_hook_(std::move(on_enter_hook)) {}

    void on_enter() override {
        if (on_enter_hook_)
            on_enter_hook_();
        belt_target_velocity_ = target_velocity_;
        belt_torque_offset_ = torque_offset_value_;
        belt_error_gain_ = error_gain_value_;
        belt_use_decel_pid_ = true;  // 启用减速PID参数
        stall_counter_ = 0;
        zero_counter_ = 0;
    }

    ActionStatus update() override {
        // 超时检测
        if (elapsed_ticks() >= timeout_ticks_) {
            return ActionStatus::SUCCESS;
        }

        // 计算平均速度和扭矩
        double avg_velocity =
            (std::abs(left_belt_velocity_) + std::abs(right_belt_velocity_)) / 2.0;
        bool torque_active = std::abs(left_belt_torque_) > stall_torque_threshold_
                          || std::abs(right_belt_torque_) > stall_torque_threshold_;

        // 最小运行帧数后才开始检测
        if (elapsed_ticks() > min_running_ticks_) {
            // 优先检测堵转：速度低且扭矩高（碰到机械限位）
            if (enable_stall_detection_) {
                if (avg_velocity < stall_velocity_threshold_ && torque_active) {
                    ++stall_counter_;
                    if (stall_counter_ >= stall_confirm_ticks_) {
                        return ActionStatus::SUCCESS;
                    }
                } else {
                    stall_counter_ = 0;
                }
            }

            // 零速检测：速度低于阈值且扭矩不高（正常减速到0）
            // 只有在没有堵转的情况下才检测零速
            if (enable_zero_velocity_detection_ && !torque_active) {
                if (avg_velocity < zero_velocity_threshold_) {
                    ++zero_counter_;
                    if (zero_counter_ >= zero_confirm_ticks_) {
                        return ActionStatus::SUCCESS;
                    }
                } else {
                    zero_counter_ = 0;
                }
            }
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        if (send_wait_zero_on_exit_ && belt_command_ != nullptr) {
            *belt_command_ = rmcs_msgs::DartSliderStatus::WAIT;
            belt_target_velocity_ = 0.0;
            if (belt_wait_zero_velocity_ != nullptr) {
                *belt_wait_zero_velocity_ = true;
            }
        }
        belt_torque_offset_ = 0.0;
        belt_error_gain_ = 1.0;
        belt_use_decel_pid_ = false;  // 恢复正常PID参数
    }

private:
    double& belt_target_velocity_;
    double& belt_torque_offset_;
    double& belt_error_gain_;
    bool& belt_use_decel_pid_;
    const double& left_belt_velocity_;
    const double& right_belt_velocity_;
    const double& left_belt_torque_;
    const double& right_belt_torque_;

    double target_velocity_;
    const double& torque_offset_value_;
    double error_gain_value_;
    bool enable_stall_detection_;
    bool enable_zero_velocity_detection_;
    double stall_velocity_threshold_;
    double stall_torque_threshold_;
    double zero_velocity_threshold_;
    uint64_t stall_confirm_ticks_;
    uint64_t zero_confirm_ticks_;
    uint64_t min_running_ticks_;
    uint64_t timeout_ticks_;
    rmcs_msgs::DartSliderStatus* belt_command_;
    bool* belt_wait_zero_velocity_;
    bool send_wait_zero_on_exit_;
    std::function<void()> on_enter_hook_;

    uint64_t stall_counter_{0};
    uint64_t zero_counter_{0};

    static constexpr double kZero_ = 0.0;
};

} // namespace rmcs_dart_guidance::manager
