#pragma once

#include <cmath>
#include <cstdint>
#include <numbers>
#include <optional>
#include <string>
#include <utility>

#include "manager/core/runtime/action.hpp"
#include "rmcs_msgs/dart_mechanism_command.hpp"

namespace rmcs_dart_guidance::manager {

class CarriageControlAction : public IAction {
public:
    CarriageControlAction(
        std::string name,                                          //
        rmcs_msgs::DartMechanismCommand& carriage_command_interface, //
        double& target_velocity_interface,                         //
        const double& carriage_velocity,                             //
        const double& carriage_torque,                               //
        rmcs_msgs::DartMechanismCommand command_setting,           //
        double velocity_setting,                                   //
        double carriage_stall_velocity_threshold,                    //
        double carriage_stall_torque_threshold,                      //
        uint64_t carriage_stall_confirm_ticks,                       //
        uint64_t timeout_ticks_setting,                              //
        const double* carriage_angle = nullptr,                      //
        std::optional<double>* carriage_reference_angle = nullptr    //
        )
        : IAction(std::move(name))
        , carriage_command_(carriage_command_interface)
        , carriage_target_velocity_(target_velocity_interface)
        , carriage_velocity_(carriage_velocity)
        , carriage_torque_(carriage_torque)
        , carriage_angle_(carriage_angle)
        , carriage_reference_angle_(carriage_reference_angle)
        , command_(command_setting)
        , target_velocity_(velocity_setting)
        , carriage_stall_velocity_threshold_(carriage_stall_velocity_threshold)
        , carriage_stall_torque_threshold_(carriage_stall_torque_threshold)
        , carriage_stall_confirm_ticks_(carriage_stall_confirm_ticks)
        , timeout_ticks_(timeout_ticks_setting) {}

    void on_enter() override {
        carriage_command_ = command_;
        carriage_target_velocity_ = target_velocity_;
        stall_counter_ = 0;
    }

    ActionStatus update() override {
        if (elapsed_ticks() >= timeout_ticks_) {
            return fail(ActionFailureReason::TIMEOUT);
        }

        if (elapsed_ticks() <= 100) {
            stall_counter_ = 0;
            return ActionStatus::RUNNING;
        }

        if (std::abs(carriage_velocity_) <= carriage_stall_velocity_threshold_
            && std::abs(carriage_torque_) >= carriage_stall_torque_threshold_) {
            ++stall_counter_;
            if (stall_counter_ >= carriage_stall_confirm_ticks_) {
                if (carriage_angle_ != nullptr && carriage_reference_angle_ != nullptr) {
                    *carriage_reference_angle_ = *carriage_angle_;
                }
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
    }

private:
    rmcs_msgs::DartMechanismCommand& carriage_command_;
    double& carriage_target_velocity_;
    const double& carriage_velocity_;
    const double& carriage_torque_;
    const double* carriage_angle_;
    std::optional<double>* carriage_reference_angle_;

    rmcs_msgs::DartMechanismCommand command_;
    double target_velocity_;
    double carriage_stall_velocity_threshold_;
    double carriage_stall_torque_threshold_;
    uint64_t carriage_stall_confirm_ticks_;
    uint64_t stall_counter_{0};
    uint64_t timeout_ticks_;
};

class CarriageTravelAction : public IAction {
public:
    CarriageTravelAction(
        std::string name,                                          //
        rmcs_msgs::DartMechanismCommand& carriage_command_interface, //
        double& target_velocity_interface,                         //
        const double& carriage_angle,                                //
        const double& carriage_velocity,                             //
        const double& carriage_torque,                               //
        rmcs_msgs::DartMechanismCommand command_setting,           //
        double velocity_setting,                                   //
        double travel_distance_setting,                            //
        uint64_t timeout_ticks_setting,                              //
        const std::optional<double>& carriage_reference_angle      //
        )
        : IAction(std::move(name))
        , carriage_command_(carriage_command_interface)
        , carriage_target_velocity_(target_velocity_interface)
        , carriage_angle_(carriage_angle)
        , carriage_velocity_(carriage_velocity)
        , carriage_torque_(carriage_torque)
        , carriage_reference_angle_(carriage_reference_angle)
        , command_(command_setting)
        , target_velocity_(velocity_setting)
        , target_travel_distance_(travel_distance_setting)
        , timeout_ticks_(timeout_ticks_setting) {}

    void on_enter() override {
        carriage_command_ = command_;
        carriage_target_velocity_ = target_velocity_;
    }

    ActionStatus update() override {
        if (elapsed_ticks() >= timeout_ticks_) {
            return fail(ActionFailureReason::TIMEOUT);
        }

        if (!carriage_reference_angle_.has_value()) {
            return fail(ActionFailureReason::INVALID_INPUT);
        }

        constexpr double kCarriageLeadMetersPerTurn = 0.002;
        const double travel_distance = std::abs(
            (carriage_angle_ - *carriage_reference_angle_) / (2.0 * std::numbers::pi)
            * kCarriageLeadMetersPerTurn);
        if (travel_distance >= target_travel_distance_) {
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
    [[maybe_unused]] const double& carriage_velocity_;
    [[maybe_unused]] const double& carriage_torque_;
    const std::optional<double>& carriage_reference_angle_;

    rmcs_msgs::DartMechanismCommand command_;
    double target_velocity_;
    double target_travel_distance_;
    uint64_t timeout_ticks_;
};
} // namespace rmcs_dart_guidance::manager
