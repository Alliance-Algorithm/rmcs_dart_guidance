#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

#include "manager/core/runtime/action.hpp"
#include "rmcs_msgs/dart_mechanism_command.hpp"

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// CarriageControlAction
//   丝杆堵转标定动作：进入时下发方向、速度和力矩上限覆盖值。
//   当丝杆在短暂起步窗口之后连续满足“低速且高扭矩”时，认为已顶到机械限位，
//   将当前编码器角度记为 origin 并返回 SUCCESS。退出时恢复 WAIT 与默认力矩设置。
// ─────────────────────────────────────────────────────────────────────────────
class CarriageControlAction : public IAction {
public:
    CarriageControlAction(
        std::string name,                                            //
        rmcs_msgs::DartMechanismCommand& carriage_command_interface, //
        double& target_velocity_interface,                           //
        double& carriage_origin_angle_interface,                     //
        double& force_max_torque_override_interface,                 //
        const double& carriage_angle,                                //
        const double& carriage_velocity,                             //
        const double& carriage_torque,                               //
        rmcs_msgs::DartMechanismCommand command_setting,             //
        double velocity_setting,                                     //
        double max_torque_override_setting,                          //
        double carriage_stall_velocity_threshold,                    //
        double carriage_stall_torque_threshold,                      //
        uint64_t carriage_stall_confirm_ticks                        //
        )
        : IAction(std::move(name))
        , carriage_command_(carriage_command_interface)
        , carriage_target_velocity_(target_velocity_interface)
        , force_max_torque_override_(force_max_torque_override_interface)
        , carriage_origin_angle_(carriage_origin_angle_interface)
        , carriage_velocity_(carriage_velocity)
        , carriage_torque_(carriage_torque)
        , carriage_angle_(carriage_angle)
        , command_(command_setting)
        , target_velocity_(velocity_setting)
        , max_torque_override_(max_torque_override_setting)
        , carriage_stall_velocity_threshold_(carriage_stall_velocity_threshold)
        , carriage_stall_torque_threshold_(carriage_stall_torque_threshold)
        , carriage_stall_confirm_ticks_(carriage_stall_confirm_ticks) {}

    void on_enter() override {
        carriage_command_ = command_;
        carriage_target_velocity_ = target_velocity_;
        force_max_torque_override_ = max_torque_override_;
        stall_counter_ = 0;
    }

    ActionStatus update() override {
        if (elapsed_ticks() <= 100) {
            stall_counter_ = 0;
            return ActionStatus::RUNNING;
        }

        if (std::abs(carriage_velocity_) <= carriage_stall_velocity_threshold_
            && std::abs(carriage_torque_) >= carriage_stall_torque_threshold_) {
            ++stall_counter_;
            if (stall_counter_ >= carriage_stall_confirm_ticks_) {
                carriage_origin_angle_ = carriage_angle_;
                return ActionStatus::SUCCESS;
            }
        } else {
            stall_counter_ = 0;
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        carriage_command_ = rmcs_msgs::DartMechanismCommand::WAIT;
        carriage_target_velocity_ = 0.0;
        force_max_torque_override_ = std::numeric_limits<double>::quiet_NaN();
    }

private:
    rmcs_msgs::DartMechanismCommand& carriage_command_;
    double& carriage_target_velocity_;
    double& force_max_torque_override_;
    double& carriage_origin_angle_;
    const double& carriage_velocity_;
    const double& carriage_torque_;
    const double& carriage_angle_;

    rmcs_msgs::DartMechanismCommand command_;
    double target_velocity_;
    double max_torque_override_;
    double carriage_stall_velocity_threshold_;
    double carriage_stall_torque_threshold_;
    uint64_t carriage_stall_confirm_ticks_;
    uint64_t stall_counter_{0};
};

// ─────────────────────────────────────────────────────────────────────────────
// CarriageTravelAction
//   丝杆定程动作：以原点角度或进入动作时的当前位置作为参考，按目标方向运动。
//   当累计位移达到目标角度时返回 SUCCESS；若运动过程中持续堵转，则返回 STALL。
//   退出时会将丝杆命令恢复到 WAIT。
// ─────────────────────────────────────────────────────────────────────────────
class CarriageTravelAction : public IAction {
public:
    // 指定位移的参考起点：可使用丝杆原点，也可使用进入动作时的当前角度。
    enum class TravelReferenceMode {
        ORIGIN_ANGLE,
        CURRENT_ANGLE,
    };

    CarriageTravelAction(
        std::string name,                                            //
        rmcs_msgs::DartMechanismCommand& carriage_command_interface, //
        double& target_velocity_interface,                           //
        const double& carriage_angle,                                //
        const double& carriage_origin_angle,                         //
        const double& carriage_velocity,                             //
        const double& carriage_torque,                               //
        rmcs_msgs::DartMechanismCommand command_setting,             //
        double velocity_setting,                                     //
        double travel_distance_setting,                              //
        double carriage_stall_velocity_threshold,                    //
        double carriage_stall_torque_threshold,                      //
        uint64_t carriage_stall_confirm_ticks,                       //
        TravelReferenceMode travel_reference_mode = TravelReferenceMode::ORIGIN_ANGLE
        //

        )
        : IAction(std::move(name))
        , carriage_command_(carriage_command_interface)
        , carriage_target_velocity_(target_velocity_interface)
        , carriage_angle_(carriage_angle)
        , carriage_origin_angle_(carriage_origin_angle)
        , carriage_velocity_(carriage_velocity)
        , carriage_torque_(carriage_torque)
        , command_(command_setting)
        , target_velocity_(velocity_setting)
        , target_travel_distance_(travel_distance_setting)
        , carriage_stall_velocity_threshold_(carriage_stall_velocity_threshold)
        , carriage_stall_torque_threshold_(carriage_stall_torque_threshold)
        , carriage_stall_confirm_ticks_(carriage_stall_confirm_ticks)
        , travel_reference_mode_(travel_reference_mode) {}

    void on_enter() override {
        carriage_command_ = command_;
        carriage_target_velocity_ = target_velocity_;
        reference_angle_ = travel_reference_mode_ == TravelReferenceMode::CURRENT_ANGLE
                             ? carriage_angle_
                             : carriage_origin_angle_;
        stall_counter_ = 0;
    }

    ActionStatus update() override {

        if (elapsed_ticks() > 200) {
            if (std::abs(carriage_velocity_) <= carriage_stall_velocity_threshold_
                && std::abs(carriage_torque_) >= carriage_stall_torque_threshold_) {
                ++stall_counter_;
                if (stall_counter_ >= carriage_stall_confirm_ticks_) {
                    return fail(ActionFailureReason::STALL);
                }
            } else {
                stall_counter_ = 0;
            }
        }

        const double travel_angle = std::abs(carriage_angle_ - reference_angle_);
        if (travel_angle >= target_travel_distance_) {
            return ActionStatus::SUCCESS;
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        carriage_command_ = rmcs_msgs::DartMechanismCommand::WAIT;
        carriage_target_velocity_ = 0.0;
    }

private:
    rmcs_msgs::DartMechanismCommand& carriage_command_;
    double& carriage_target_velocity_;
    const double& carriage_angle_;
    const double& carriage_origin_angle_;
    [[maybe_unused]] const double& carriage_velocity_;
    [[maybe_unused]] const double& carriage_torque_;

    rmcs_msgs::DartMechanismCommand command_;
    double target_velocity_;
    double target_travel_distance_;
    double carriage_stall_velocity_threshold_;
    double carriage_stall_torque_threshold_;
    uint64_t carriage_stall_confirm_ticks_;
    TravelReferenceMode travel_reference_mode_;
    double reference_angle_{0.0};
    uint64_t stall_counter_{0};
};

// ─────────────────────────────────────────────────────────────────────────────
// CarriageAngleCloseLoopAction
//   丝杆角度闭环动作：进入时根据原点和目标方向计算目标角度，并写给下层闭环控制。
//   在最小运行时间之后，当当前角度进入允许误差范围即返回 SUCCESS；超时则返回
//   TIMEOUT。退出时清除目标角度并恢复 WAIT。
// ─────────────────────────────────────────────────────────────────────────────
class CarriageAngleCloseLoopAction : public IAction {
public:
    CarriageAngleCloseLoopAction(
        std::string name,                                            //
        rmcs_msgs::DartMechanismCommand& carriage_command_interface, //
        double& target_velocity_interface,                           //
        double& target_angle_interface,                              //
        const double& carriage_angle,                                //
        const double& carriage_origin_angle,                         //
        rmcs_msgs::DartMechanismCommand command_setting,             //
        double velocity_setting,                                     //
        double travel_angle_setting,                                 //
        double allowable_error_setting,                              //
        uint64_t min_run_ticks_setting,                              //
        uint64_t timeout_ticks_setting                               //
        )
        : IAction(std::move(name))
        , carriage_command_(carriage_command_interface)
        , carriage_target_velocity_(target_velocity_interface)
        , carriage_target_angle_(target_angle_interface)
        , carriage_angle_(carriage_angle)
        , carriage_origin_angle_(carriage_origin_angle)
        , command_(command_setting)
        , target_velocity_(velocity_setting)
        , target_travel_angle_(travel_angle_setting)
        , allowable_error_(allowable_error_setting)
        , min_run_ticks_(min_run_ticks_setting)
        , timeout_ticks_(timeout_ticks_setting) {}

    void on_enter() override {
        carriage_command_ = command_;
        carriage_target_velocity_ = target_velocity_;
        target_angle_ = compute_target_angle();
        carriage_target_angle_ = target_angle_;
    }

    ActionStatus update() override {
        if (elapsed_ticks() >= timeout_ticks_) {
            return fail(ActionFailureReason::TIMEOUT);
        }

        if (elapsed_ticks() < min_run_ticks_) {
            return ActionStatus::RUNNING;
        }

        if (std::abs(carriage_angle_ - target_angle_) <= allowable_error_) {
            return ActionStatus::SUCCESS;
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        carriage_command_ = rmcs_msgs::DartMechanismCommand::WAIT;
        carriage_target_velocity_ = 0.0;
        carriage_target_angle_ = std::numeric_limits<double>::quiet_NaN();
    }

private:
    double compute_target_angle() const {
        switch (command_) {
        case rmcs_msgs::DartMechanismCommand::DOWN:
            return carriage_origin_angle_ + target_travel_angle_;
        case rmcs_msgs::DartMechanismCommand::UP:
            return carriage_origin_angle_ - target_travel_angle_;
        default: return carriage_origin_angle_;
        }
    }

    rmcs_msgs::DartMechanismCommand& carriage_command_;
    double& carriage_target_velocity_;
    double& carriage_target_angle_;
    const double& carriage_angle_;
    const double& carriage_origin_angle_;

    rmcs_msgs::DartMechanismCommand command_;
    double target_velocity_;
    double target_travel_angle_;
    double allowable_error_;
    uint64_t min_run_ticks_;
    uint64_t timeout_ticks_;
    double target_angle_{0.0};
};
} // namespace rmcs_dart_guidance::manager
