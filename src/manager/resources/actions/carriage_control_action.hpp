#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <utility>

#include "manager/core/runtime/action.hpp"
#include "manager/resources/carriage_origin_state.hpp"
#include "rmcs_msgs/dart_mechanism_command.hpp"

namespace rmcs_dart_guidance::manager {

class CarriageControlAction : public IAction {
public:
    CarriageControlAction(
        std::string name,                                          //
        rmcs_msgs::DartMechanismCommand& carriage_command_interface, //
        double& target_velocity_interface,                         //
        double& force_max_torque_override_interface,                //
        const double& carriage_velocity,                             //
        const double& carriage_torque,                               //
        rmcs_msgs::DartMechanismCommand command_setting,           //
        double velocity_setting,                                   //
        double max_torque_override_setting,                          //
        double carriage_stall_velocity_threshold,                    //
        double carriage_stall_torque_threshold,                      //
        uint64_t carriage_stall_confirm_ticks,                       //
        uint64_t timeout_ticks_setting,                              //
        const double* carriage_angle = nullptr,                      //
        std::optional<double>* carriage_origin_angle = nullptr,      //
        bool persist_origin_on_success = true                        //
        )
        : IAction(std::move(name))
        , carriage_command_(carriage_command_interface)
        , carriage_target_velocity_(target_velocity_interface)
        , force_max_torque_override_(force_max_torque_override_interface)
        , carriage_velocity_(carriage_velocity)
        , carriage_torque_(carriage_torque)
        , carriage_angle_(carriage_angle)
        , carriage_origin_angle_(carriage_origin_angle)
        , command_(command_setting)
        , target_velocity_(velocity_setting)
        , max_torque_override_(max_torque_override_setting)
        , carriage_stall_velocity_threshold_(carriage_stall_velocity_threshold)
        , carriage_stall_torque_threshold_(carriage_stall_torque_threshold)
        , carriage_stall_confirm_ticks_(carriage_stall_confirm_ticks)
        , timeout_ticks_(timeout_ticks_setting)
        , persist_origin_on_success_(persist_origin_on_success) {}

    void on_enter() override {
        carriage_command_ = command_;
        carriage_target_velocity_ = target_velocity_;
        force_max_torque_override_ = max_torque_override_;
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
                if (carriage_angle_ != nullptr && carriage_origin_angle_ != nullptr) {
                    const double carriage_origin_angle = *carriage_angle_;
                    *carriage_origin_angle_ = carriage_origin_angle;
                    if (persist_origin_on_success_) {
                        std::string error_message;
                        if (!store_carriage_power_cycle_origin(
                                carriage_origin_angle, &error_message)) {
                            if (runtime_context().logger != nullptr) {
                                RCLCPP_ERROR(
                                    *runtime_context().logger,
                                    "[CarriageControlAction] failed to persist carriage origin: %s",
                                    error_message.c_str());
                            }
                            return fail(ActionFailureReason::DEPENDENCY_FAILURE);
                        }
                    }
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
        force_max_torque_override_ = std::numeric_limits<double>::quiet_NaN();
    }

private:
    rmcs_msgs::DartMechanismCommand& carriage_command_;
    double& carriage_target_velocity_;
    double& force_max_torque_override_;
    const double& carriage_velocity_;
    const double& carriage_torque_;
    const double* carriage_angle_;
    std::optional<double>* carriage_origin_angle_;

    rmcs_msgs::DartMechanismCommand command_;
    double target_velocity_;
    double max_torque_override_;
    double carriage_stall_velocity_threshold_;
    double carriage_stall_torque_threshold_;
    uint64_t carriage_stall_confirm_ticks_;
    uint64_t stall_counter_{0};
    uint64_t timeout_ticks_;
    bool persist_origin_on_success_;
};

class CarriageTravelAction : public IAction {
public:
    enum class TravelReferenceMode {
        ORIGIN_ABSOLUTE,
        CURRENT_RELATIVE,
    };

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
        double carriage_stall_velocity_threshold,                    //
        double carriage_stall_torque_threshold,                      //
        uint64_t carriage_stall_confirm_ticks,                       //
        uint64_t timeout_ticks_setting,                              //
        const std::optional<double>& carriage_origin_angle,         //
        TravelReferenceMode travel_reference_mode =
            TravelReferenceMode::ORIGIN_ABSOLUTE                    //
        )
        : IAction(std::move(name))
        , carriage_command_(carriage_command_interface)
        , carriage_target_velocity_(target_velocity_interface)
        , carriage_angle_(carriage_angle)
        , carriage_velocity_(carriage_velocity)
        , carriage_torque_(carriage_torque)
        , carriage_origin_angle_(carriage_origin_angle)
        , command_(command_setting)
        , target_velocity_(velocity_setting)
        , target_travel_distance_(travel_distance_setting)
        , carriage_stall_velocity_threshold_(carriage_stall_velocity_threshold)
        , carriage_stall_torque_threshold_(carriage_stall_torque_threshold)
        , carriage_stall_confirm_ticks_(carriage_stall_confirm_ticks)
        , timeout_ticks_(timeout_ticks_setting)
        , travel_reference_mode_(travel_reference_mode) {}

    void on_enter() override {
        carriage_command_ = command_;
        carriage_target_velocity_ = target_velocity_;
        stall_counter_ = 0;
        start_angle_ = carriage_angle_;
    }

    ActionStatus update() override {
        if (elapsed_ticks() >= timeout_ticks_) {
            return fail(ActionFailureReason::TIMEOUT);
        }

        if (travel_reference_mode_ == TravelReferenceMode::ORIGIN_ABSOLUTE
            && !carriage_origin_angle_.has_value()) {
            return fail(ActionFailureReason::INVALID_INPUT);
        }

        if (elapsed_ticks() > 100) {
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

        constexpr double kCarriageLeadMetersPerTurn = 0.002;
        const double reference_angle =
            travel_reference_mode_ == TravelReferenceMode::CURRENT_RELATIVE
                ? start_angle_
                : *carriage_origin_angle_;
        const double travel_distance =
            std::abs((carriage_angle_ - reference_angle) / (2.0 * std::numbers::pi)
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
    const std::optional<double>& carriage_origin_angle_;

    rmcs_msgs::DartMechanismCommand command_;
    double target_velocity_;
    double target_travel_distance_;
    double carriage_stall_velocity_threshold_;
    double carriage_stall_torque_threshold_;
    uint64_t carriage_stall_confirm_ticks_;
    uint64_t stall_counter_{0};
    uint64_t timeout_ticks_;
    TravelReferenceMode travel_reference_mode_;
    double start_angle_{0.0};
};
} // namespace rmcs_dart_guidance::manager
