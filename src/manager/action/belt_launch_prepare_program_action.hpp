#pragma once

#include "action.hpp"

#include <cmath>
#include <cstdint>
#include <string>

#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

// BeltLaunchPrepareProgramAction
//   触发 controller 内部的传送带阶段程序（LaunchPreparationTask 专用），并在 manager 侧基于反馈等待其完成。
//
// 说明：
//   - 为避免 rmcs_executor 的循环依赖（manager <-> controller），该 Action 不读取 controller 的 program_done。
//   - 完成条件严格复刻此前 manager 内 BeltDecelerationAction 的检测逻辑（零速/堵转）。
//   - 速度/距离/切换点等阶段字面量由 controller 内部实现；manager 仅注入力矩补偿与力矩限幅。
class BeltLaunchPrepareProgramAction : public IAction {
public:
    enum class Program : uint8_t {
        LAUNCH_PREP_DOWN = 1,
        LAUNCH_PREP_UP = 2,
    };

    BeltLaunchPrepareProgramAction(
        std::string name, uint8_t& belt_program_cmd, bool& belt_program_first_shot,
        double& belt_program_down_comp_torque, double& belt_program_up_comp_torque,
        double& belt_torque_limit, rmcs_msgs::DartSliderStatus& belt_command,
        double& belt_target_velocity, double& belt_hold_torque, bool& belt_wait_zero_velocity,
        double& belt_torque_offset, double& belt_error_gain, bool& belt_use_decel_pid,
        const double& left_belt_velocity, const double& right_belt_velocity,
        const double& left_belt_torque, const double& right_belt_torque, Program program,
        bool first_shot, double down_comp_torque_value, double up_comp_torque_value,
        double torque_limit_value, uint64_t timeout_ticks)
        : IAction(std::move(name))
        , belt_program_cmd_(belt_program_cmd)
        , belt_program_first_shot_(belt_program_first_shot)
        , belt_program_down_comp_torque_(belt_program_down_comp_torque)
        , belt_program_up_comp_torque_(belt_program_up_comp_torque)
        , belt_torque_limit_(belt_torque_limit)
        , belt_command_(belt_command)
        , belt_target_velocity_(belt_target_velocity)
        , belt_hold_torque_(belt_hold_torque)
        , belt_wait_zero_velocity_(belt_wait_zero_velocity)
        , belt_torque_offset_(belt_torque_offset)
        , belt_error_gain_(belt_error_gain)
        , belt_use_decel_pid_(belt_use_decel_pid)
        , left_belt_velocity_(left_belt_velocity)
        , right_belt_velocity_(right_belt_velocity)
        , left_belt_torque_(left_belt_torque)
        , right_belt_torque_(right_belt_torque)
        , program_(program)
        , first_shot_(first_shot)
        , down_comp_torque_value_(down_comp_torque_value)
        , up_comp_torque_value_(up_comp_torque_value)
        , torque_limit_value_(torque_limit_value)
        , timeout_ticks_(timeout_ticks) {}

    void on_enter() override {
        belt_program_cmd_ = static_cast<uint8_t>(program_);
        belt_program_first_shot_ = first_shot_;
        belt_program_down_comp_torque_ = down_comp_torque_value_;
        belt_program_up_comp_torque_ = up_comp_torque_value_;
        belt_torque_limit_ = torque_limit_value_;

        stall_counter_ = 0;
        zero_counter_ = 0;
    }

    ActionStatus update() override {
        if (elapsed_ticks() >= timeout_ticks_) {
            return ActionStatus::SUCCESS;
        }

        const double avg_velocity =
            (std::abs(left_belt_velocity_) + std::abs(right_belt_velocity_)) / 2.0;

        if (program_ == Program::LAUNCH_PREP_DOWN) {
            return update_down(avg_velocity);
        }

        return update_up(avg_velocity);
    }

    void on_exit() override {
        belt_program_cmd_ = 0;

        if (program_ == Program::LAUNCH_PREP_UP) {
            // 复刻原 BeltDecelerationAction(send_wait_zero_on_exit=true) 的退出效果：
            // 1) 显式下发 WAIT/0
            // 2) 切换到 WAIT_ZERO（零速闭环）
            // 3) 清理 torque_offset/error_gain/use_decel_pid
            belt_command_ = rmcs_msgs::DartSliderStatus::WAIT;
            belt_target_velocity_ = 0.0;
            belt_hold_torque_ = 0.0;
            belt_wait_zero_velocity_ = true;
            belt_torque_offset_ = 0.0;
            belt_error_gain_ = 1.0;
            belt_use_decel_pid_ = false;
            return;
        }

        // 下行程序结束后下一步立刻进入 hold/lock；这里仅做最小清理，避免残留。
        belt_torque_offset_ = 0.0;
        belt_error_gain_ = 1.0;
        belt_use_decel_pid_ = false;
        belt_wait_zero_velocity_ = false;
    }

private:
    ActionStatus update_down(double avg_velocity) {
        // 复刻 LaunchPreparationTask 内 BeltDecelerationAction 的检测参数（下行减速）：
        // - stall: vel < 0.3 & torque_active, confirm 50
        // - zero : vel < 0.05 & !torque_active, confirm 60
        // - min_running_ticks: 50
        static constexpr uint64_t kMinRunningTicks = 50;
        static constexpr double kStallVelocityThreshold = 0.3;
        static constexpr double kStallTorqueThreshold = 1.0;
        static constexpr uint64_t kStallConfirmTicks = 50;
        static constexpr double kZeroVelocityThreshold = 0.05;
        static constexpr uint64_t kZeroConfirmTicks = 60;

        const bool torque_active =
            std::abs(left_belt_torque_) > kStallTorqueThreshold
            || std::abs(right_belt_torque_) > kStallTorqueThreshold;

        if (elapsed_ticks() > kMinRunningTicks) {
            if (avg_velocity < kStallVelocityThreshold && torque_active) {
                ++stall_counter_;
                if (stall_counter_ >= kStallConfirmTicks) {
                    return ActionStatus::SUCCESS;
                }
            } else {
                stall_counter_ = 0;
            }

            if (!torque_active) {
                if (avg_velocity < kZeroVelocityThreshold) {
                    ++zero_counter_;
                    if (zero_counter_ >= kZeroConfirmTicks) {
                        return ActionStatus::SUCCESS;
                    }
                } else {
                    zero_counter_ = 0;
                }
            } else {
                zero_counter_ = 0;
            }
        }

        return ActionStatus::RUNNING;
    }

    ActionStatus update_up(double avg_velocity) {
        // 复刻 LaunchPreparationTask 内 BeltDecelerationAction 的检测参数（上行减速+堵转）：
        // - stall: vel < 0.15 & torque_active, confirm 100
        // - min_running_ticks: 300
        static constexpr uint64_t kMinRunningTicks = 300;
        static constexpr double kStallVelocityThreshold = 0.15;
        static constexpr double kStallTorqueThreshold = 0.5;
        static constexpr uint64_t kStallConfirmTicks = 100;

        const bool torque_active =
            std::abs(left_belt_torque_) > kStallTorqueThreshold
            || std::abs(right_belt_torque_) > kStallTorqueThreshold;

        if (elapsed_ticks() > kMinRunningTicks) {
            if (avg_velocity < kStallVelocityThreshold && torque_active) {
                ++stall_counter_;
                if (stall_counter_ >= kStallConfirmTicks) {
                    return ActionStatus::SUCCESS;
                }
            } else {
                stall_counter_ = 0;
            }
        }

        return ActionStatus::RUNNING;
    }

    uint8_t& belt_program_cmd_;
    bool& belt_program_first_shot_;
    double& belt_program_down_comp_torque_;
    double& belt_program_up_comp_torque_;
    double& belt_torque_limit_;

    rmcs_msgs::DartSliderStatus& belt_command_;
    double& belt_target_velocity_;
    double& belt_hold_torque_;
    bool& belt_wait_zero_velocity_;
    double& belt_torque_offset_;
    double& belt_error_gain_;
    bool& belt_use_decel_pid_;

    const double& left_belt_velocity_;
    const double& right_belt_velocity_;
    const double& left_belt_torque_;
    const double& right_belt_torque_;

    Program program_;
    bool first_shot_;
    double down_comp_torque_value_;
    double up_comp_torque_value_;
    double torque_limit_value_;
    uint64_t timeout_ticks_;

    uint64_t stall_counter_{0};
    uint64_t zero_counter_{0};
};

} // namespace rmcs_dart_guidance::manager

