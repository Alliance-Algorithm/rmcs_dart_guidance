#pragma once

#include <cstdlib>
#include <functional>
#include <memory>
#include <numeric>
#include <vector>

#include <rclcpp/logging.hpp>

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/carriage_control_action.hpp"
#include "manager/resources/carriage_origin_state.hpp"

namespace rmcs_dart_guidance::manager {

class CallbackAction : public IAction {
public:
    using Callback = std::function<ActionStatus(CallbackAction&)>;

    CallbackAction(std::string name, Callback callback)
        : IAction(std::move(name))
        , callback_(std::move(callback)) {}

    ActionStatus update() override { return callback_(*this); }

    ActionStatus fail_with(ActionFailureReason reason) { return fail(reason); }

private:
    Callback callback_;
};

class CarriageInitTask : public Task {
public:
    CarriageInitTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings, ManagerRuntimeState& runtime_state)
        : Task("carriage_init", "丝杆低速堵转初始化") {
        constexpr std::size_t kCalibrationPassCount = 3;
        auto carriage_origin_samples = std::make_shared<std::vector<double>>();
        carriage_origin_samples->reserve(kCalibrationPassCount);

        auto make_record_action = [&runtime_state, carriage_origin_samples]() {
            return std::make_shared<CallbackAction>(
                "carriage_origin_record",
                [&runtime_state, carriage_origin_samples](CallbackAction& action) {
                    if (!runtime_state.carriage_power_cycle_origin_angle.has_value()) {
                        return action.fail_with(ActionFailureReason::INVALID_INPUT);
                    }
                    carriage_origin_samples->push_back(
                        *runtime_state.carriage_power_cycle_origin_angle);
                    return ActionStatus::SUCCESS;
                });
        };

        auto make_average_action = [&runtime_state, carriage_origin_samples]() {
            return std::make_shared<CallbackAction>(
                "carriage_origin_average",
                [&runtime_state, carriage_origin_samples](CallbackAction& action) {
                    if (carriage_origin_samples == nullptr
                        || carriage_origin_samples->size() != kCalibrationPassCount) {
                        return action.fail_with(ActionFailureReason::INVALID_INPUT);
                    }

                    const double average_angle =
                        std::accumulate(
                            carriage_origin_samples->begin(), carriage_origin_samples->end(), 0.0)
                        / static_cast<double>(carriage_origin_samples->size());
                    runtime_state.carriage_power_cycle_origin_angle = average_angle;

                    std::string error_message;
                    if (!store_carriage_power_cycle_origin(average_angle, &error_message)) {
                        if (action.runtime_context().logger != nullptr) {
                            RCLCPP_ERROR(
                                *action.runtime_context().logger,
                                "[carriage_origin_average] failed to persist averaged origin: %s",
                                error_message.c_str());
                        }
                        return action.fail_with(ActionFailureReason::DEPENDENCY_FAILURE);
                    }

                    return ActionStatus::SUCCESS;
                });
        };

        for (std::size_t index = 0; index < kCalibrationPassCount; ++index) {
            then(
                std::make_shared<CarriageControlAction>(
                    "carriage_init", output.carriage_command, output.carriage_target_velocity,
                    output.force_max_torque_override, input.carriage_velocity,
                    input.carriage_torque, rmcs_msgs::DartMechanismCommand::UP,
                    settings.carriage_calibration_setting_velocity,
                    settings.carriage_calibration_max_torque,
                    settings.carriage_calibration_stall_velocity_threshold,
                    settings.carriage_calibration_stall_torque_threshold,
                    settings.carriage_calibration_stall_confirm_ticks, 20000, &input.carriage_angle,
                    &runtime_state.carriage_power_cycle_origin_angle, false));
            then(make_record_action());
            if (index + 1 == kCalibrationPassCount) {
                then(make_average_action());
            } else {
                then(
                    std::make_shared<CarriageTravelAction>(
                        "carriage_travel", output.carriage_command, output.carriage_target_velocity,
                        input.carriage_angle, input.carriage_velocity, input.carriage_torque,
                        rmcs_msgs::DartMechanismCommand::DOWN, settings.carriage_down_setting_velocity,
                        0.01, settings.carriage_stall_velocity_threshold,
                        settings.carriage_stall_torque_threshold,
                        settings.carriage_stall_confirm_ticks, 20000,
                        runtime_state.carriage_power_cycle_origin_angle));
            }
        }
    }
};

} // namespace rmcs_dart_guidance::manager
