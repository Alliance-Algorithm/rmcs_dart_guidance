#pragma once

#include "manager/core/runtime/action.hpp"
#include "rmcs_msgs/dart_mechanism_command.hpp"
#include "rmcs_msgs/dart_motor_exit_mode.hpp"

#include <cmath>
#include <cstdint>
#include <string>

#include <utility>

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// FillingLiftAction
//   填装升降机构开环控制动作：进入时写入升降方向与目标速度。
//   运行中通过“低速且高扭矩”连续出现来判定机构已顶到限位，满足确认次数后返回
//   SUCCESS；超时则返回 TIMEOUT。退出时根据 exit_mode 决定保持输出还是切回 WAIT。
// ─────────────────────────────────────────────────────────────────────────────
class FillingLiftAction : public IAction {
public:
    FillingLiftAction(
        std::string name,                                        //
        rmcs_msgs::DartMechanismCommand& lift_command_interface, //
        double& target_velocity_interface,                       //
        rmcs_msgs::ExitMode& exit_mode_interface,                //
        const double& lift_left_velocity,                        //
        const double& lift_left_torque,                          //
        const double& lift_right_velocity,                       //
        const double& lift_right_torque,                         //
        rmcs_msgs::DartMechanismCommand command_setting,         //
        double velocity_setting,                                 //
        rmcs_msgs::ExitMode exit_mode_setting,                   //
        double lift_stall_velocity_threshold,                    //
        double lift_stall_torque_threshold,                      //
        uint64_t lift_stall_confirm_ticks,                       //
        uint64_t timeout_ticks_setting                           //
        )
        : IAction(std::move(name))
        , lift_command_(lift_command_interface)
        , lift_target_velocity_(target_velocity_interface)
        , lift_exit_mode_(exit_mode_interface)
        , lift_left_velocity_(lift_left_velocity)
        , lift_left_torque_(lift_left_torque)
        , lift_right_velocity_(lift_right_velocity)
        , lift_right_torque_(lift_right_torque)
        , command_(command_setting)
        , target_velocity_(velocity_setting)
        , exit_mode_(exit_mode_setting)
        , lift_stall_velocity_threshold_(lift_stall_velocity_threshold)
        , lift_stall_torque_threshold_(lift_stall_torque_threshold)
        , lift_stall_confirm_ticks_(lift_stall_confirm_ticks)
        , timeout_ticks_(timeout_ticks_setting) {}

    void on_enter() override {
        lift_command_ = command_;
        lift_target_velocity_ = target_velocity_;
        stall_counter_ = 0;
    }

    ActionStatus update() override {
        if (elapsed_ticks() >= timeout_ticks_) {
            return fail(ActionFailureReason::TIMEOUT);
        }

        if (elapsed_ticks() <= 500) {
            stall_counter_ = 0;
            return ActionStatus::RUNNING;
        }

        const double avg_velocity =
            (std::abs(lift_left_velocity_) + std::abs(lift_right_velocity_)) / 2.0;
        const bool torque_active = std::abs(lift_left_torque_) > lift_stall_torque_threshold_
                                || std::abs(lift_right_torque_) > lift_stall_torque_threshold_;

        if (avg_velocity < lift_stall_velocity_threshold_ && torque_active) {
            ++stall_counter_;
            if (stall_counter_ >= lift_stall_confirm_ticks_) {
                return ActionStatus::SUCCESS;
            }
        } else {
            stall_counter_ = 0;
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        if (exit_mode_ == rmcs_msgs::ExitMode::KEEP) {
            return;
        }
        lift_command_ = rmcs_msgs::DartMechanismCommand::WAIT;
        lift_target_velocity_ = 0.0;
        lift_exit_mode_ = exit_mode_;
    }

private:
    rmcs_msgs::DartMechanismCommand& lift_command_;
    double& lift_target_velocity_;
    rmcs_msgs::ExitMode& lift_exit_mode_;
    const double& lift_left_velocity_;
    const double& lift_left_torque_;
    const double& lift_right_velocity_;
    const double& lift_right_torque_;

    rmcs_msgs::DartMechanismCommand command_;
    double target_velocity_;
    rmcs_msgs::ExitMode exit_mode_;
    double lift_stall_velocity_threshold_;
    double lift_stall_torque_threshold_;
    uint64_t lift_stall_confirm_ticks_;
    uint64_t stall_counter_{0};
    uint64_t timeout_ticks_;
};

class FillInitialFlagAction : public IAction {
public:
    FillInitialFlagAction(
        std::string name,       //
        bool& lift_initial_flag //
        )
        : IAction(std::move(name))
        , lift_initial_flag_(lift_initial_flag) {}

    void on_enter() override { lift_initial_flag_ = true; }

    ActionStatus update() override {
        if (elapsed_ticks() <= 10) {
            return ActionStatus::RUNNING;
        } else {
            return ActionStatus::SUCCESS;
        }
    }

    void on_exit() override { lift_initial_flag_ = false; }

private:
    bool& lift_initial_flag_;
};

} // namespace rmcs_dart_guidance::manager
