#pragma once

#include "manager/core/runtime/action.hpp"
#include "rmcs_msgs/dart_mechanism_command.hpp"
#include "rmcs_msgs/dart_motor_exit_mode.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include <utility>

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// BeltControlAction
//   同步带开环控制动作：进入时写入方向、目标速度和可选的力矩上限覆盖值。
//   运行中通过“低速且高扭矩”连续出现来判定堵转，达到确认次数后返回 SUCCESS，
//   超时则返回 TIMEOUT。退出时根据 exit_mode 决定保持输出还是回到 WAIT。
// ─────────────────────────────────────────────────────────────────────────────
class BeltControlAction : public IAction {
public:
    BeltControlAction(
        std::string name,                                        //
        rmcs_msgs::DartMechanismCommand& belt_command_interface, //
        double& target_velocity_interface,                       //
        rmcs_msgs::ExitMode& exit_mode_interface,                //
        double& max_torque_override_interface,                   //
        const double& belt_left_velocity,                        //
        const double& belt_left_torque,                          //
        const double& belt_right_velocity,                       //
        const double& belt_right_torque,                         //
        rmcs_msgs::DartMechanismCommand command_setting,         //
        double velocity_setting,                                 //
        rmcs_msgs::ExitMode exit_mode_setting,                   //
        double belt_stall_velocity_threshold,                    //
        double belt_stall_torque_threshold,                      //
        uint64_t belt_stall_confirm_ticks,                       //
        uint64_t timeout_ticks_setting,                          //
        double max_torque_override_setting =
            std::numeric_limits<double>::quiet_NaN()             //
        )
        : IAction(std::move(name))
        , belt_command_(belt_command_interface)
        , belt_target_velocity_(target_velocity_interface)
        , belt_exit_mode_(exit_mode_interface)
        , belt_max_torque_override_(max_torque_override_interface)
        , belt_left_velocity_(belt_left_velocity)
        , belt_left_torque_(belt_left_torque)
        , belt_right_velocity_(belt_right_velocity)
        , belt_right_torque_(belt_right_torque)
        , command_(command_setting)
        , target_velocity_(velocity_setting)
        , exit_mode_(exit_mode_setting)
        , belt_stall_velocity_threshold_(belt_stall_velocity_threshold)
        , belt_stall_torque_threshold_(belt_stall_torque_threshold)
        , belt_stall_confirm_ticks_(belt_stall_confirm_ticks)
        , timeout_ticks_(timeout_ticks_setting)
        , max_torque_override_(max_torque_override_setting) {}

    void on_enter() override {
        belt_command_ = command_;
        belt_target_velocity_ = target_velocity_;
        belt_max_torque_override_ = max_torque_override_;
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
            (std::abs(belt_left_velocity_) + std::abs(belt_right_velocity_)) / 2.0;
        const bool torque_active = std::abs(belt_left_torque_) > belt_stall_torque_threshold_
                                || std::abs(belt_right_torque_) > belt_stall_torque_threshold_;

        if (avg_velocity < belt_stall_velocity_threshold_ && torque_active) {
            ++stall_counter_;
            if (stall_counter_ >= belt_stall_confirm_ticks_) {
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
        belt_command_ = rmcs_msgs::DartMechanismCommand::WAIT;
        belt_target_velocity_ = 0.0;
        belt_exit_mode_ = exit_mode_;
        belt_max_torque_override_ = std::numeric_limits<double>::quiet_NaN();
    }

private:
    rmcs_msgs::DartMechanismCommand& belt_command_;
    double& belt_target_velocity_;
    rmcs_msgs::ExitMode& belt_exit_mode_;
    double& belt_max_torque_override_;
    const double& belt_left_velocity_;
    const double& belt_left_torque_;
    const double& belt_right_velocity_;
    const double& belt_right_torque_;

    rmcs_msgs::DartMechanismCommand command_;
    double target_velocity_;
    rmcs_msgs::ExitMode exit_mode_;
    double belt_stall_velocity_threshold_;
    double belt_stall_torque_threshold_;
    uint64_t belt_stall_confirm_ticks_;
    uint64_t stall_counter_{0};
    uint64_t timeout_ticks_;
    double max_torque_override_;
};

// ─────────────────────────────────────────────────────────────────────────────
// BeltTravelAction
//   同步带定程动作：记录进入动作时左右编码器角度，并持续统计平均行程。
//   当平均行程达到目标距离时返回 SUCCESS；若在规定时间内未达到则返回 TIMEOUT。
//   退出时根据 exit_mode 决定保持当前状态还是切回 WAIT。
// ─────────────────────────────────────────────────────────────────────────────
class BeltTravelAction : public IAction {
public:
    BeltTravelAction(
        std::string name,                                        //
        rmcs_msgs::DartMechanismCommand& belt_command_interface, //
        double& target_velocity_interface,                       //
        rmcs_msgs::ExitMode& exit_mode_interface,                //
        const double& belt_left_angle,                           //
        const double& belt_left_velocity,                        //
        const double& belt_left_torque,                          //
        const double& belt_right_angle,                          //
        const double& belt_right_velocity,                       //
        const double& belt_right_torque,                         //
        rmcs_msgs::DartMechanismCommand command_setting,         //
        double velocity_setting,                                 //
        rmcs_msgs::ExitMode exit_mode_setting,                   //
        double travel_distance_setting,                          //
        uint64_t timeout_ticks_setting                           //
        )
        : IAction(std::move(name))
        , belt_command_(belt_command_interface)
        , belt_target_velocity_(target_velocity_interface)
        , belt_exit_mode_(exit_mode_interface)
        , belt_left_angle_(belt_left_angle)
        , belt_left_velocity_(belt_left_velocity)
        , belt_left_torque_(belt_left_torque)
        , belt_right_angle_(belt_right_angle)
        , belt_right_velocity_(belt_right_velocity)
        , belt_right_torque_(belt_right_torque)
        , command_(command_setting)
        , target_velocity_(velocity_setting)
        , exit_mode_(exit_mode_setting)
        , target_travel_distance_(travel_distance_setting)
        , timeout_ticks_(timeout_ticks_setting) {}

    void on_enter() override {
        belt_command_ = command_;
        belt_target_velocity_ = target_velocity_;
        belt_left_start_angle_ = belt_left_angle_;
        belt_right_start_angle_ = belt_right_angle_;
    }

    ActionStatus update() override {
        if (elapsed_ticks() >= timeout_ticks_) {
            return fail(ActionFailureReason::TIMEOUT);
        }

        if (target_travel_distance_ <= 0.0) {
            return ActionStatus::SUCCESS;
        }

        const double left_travel_distance = std::abs(belt_left_angle_ - belt_left_start_angle_);
        const double right_travel_distance = std::abs(belt_right_angle_ - belt_right_start_angle_);
        const double average_travel_distance =
            (left_travel_distance + right_travel_distance) / 2.0;

        if (average_travel_distance >= target_travel_distance_) {
            return ActionStatus::SUCCESS;
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        if (exit_mode_ == rmcs_msgs::ExitMode::KEEP) {
            return;
        }
        belt_command_ = rmcs_msgs::DartMechanismCommand::WAIT;
        belt_target_velocity_ = 0.0;
        belt_exit_mode_ = exit_mode_;
    }

private:
    rmcs_msgs::DartMechanismCommand& belt_command_;
    double& belt_target_velocity_;
    rmcs_msgs::ExitMode& belt_exit_mode_;
    const double& belt_left_angle_;
    [[maybe_unused]] const double& belt_left_velocity_;
    [[maybe_unused]] const double& belt_left_torque_;
    const double& belt_right_angle_;
    [[maybe_unused]] const double& belt_right_velocity_;
    [[maybe_unused]] const double& belt_right_torque_;

    rmcs_msgs::DartMechanismCommand command_;
    double target_velocity_;
    rmcs_msgs::ExitMode exit_mode_;
    double target_travel_distance_;
    double belt_left_start_angle_{0.0};
    double belt_right_start_angle_{0.0};
    uint64_t timeout_ticks_;
};

} // namespace rmcs_dart_guidance::manager
