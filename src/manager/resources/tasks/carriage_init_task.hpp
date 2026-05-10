#pragma once

#include <functional>
#include <memory>
#include <numeric>

#include <rclcpp/logging.hpp>

#include "manager/core/runtime/action.hpp"
#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/carriage_control_action.hpp"
#include "manager/resources/actions/delay_action.hpp"

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
        constexpr double kCarriageCalibrationReturnEncoderAngle = 778240.0;
        auto carriage_origin_angle_interface = &output.carriage_origin_angle;

        auto make_calibration_action = [&input, &output, &settings]() {
            return std::make_shared<CarriageControlAction>(
                "carriage_init", output.carriage_command, output.carriage_target_velocity,
                output.carriage_origin_angle, output.force_max_torque_override,
                input.carriage_angle, input.carriage_velocity, input.carriage_torque,
                rmcs_msgs::DartMechanismCommand::DOWN, settings.carriage_calibration_velocity,
                settings.carriage_calibration_max_torque,
                settings.carriage_calibration_stall_velocity_threshold,
                settings.carriage_calibration_stall_torque_threshold,
                settings.carriage_calibration_stall_confirm_ticks);
        };

        auto make_return_action = [&input, &output, &settings,
                                   kCarriageCalibrationReturnEncoderAngle]() {
            return std::make_shared<CarriageTravelAction>(
                "carriage_travel", output.carriage_command, output.carriage_target_velocity,
                input.carriage_angle, input.carriage_origin_angle, input.carriage_velocity,
                input.carriage_torque, rmcs_msgs::DartMechanismCommand::UP,
                settings.carriage_up_setting_velocity, kCarriageCalibrationReturnEncoderAngle,
                settings.carriage_stall_velocity_threshold,
                settings.carriage_stall_torque_threshold, settings.carriage_stall_confirm_ticks,
                CarriageTravelAction::TravelReferenceMode::CURRENT_ANGLE);
        };

        then(make_calibration_action());
        then(std::make_shared<DelayAction>("delay", 100));
        then(
            std::make_shared<CallbackAction>(
                "carriage_origin_record",
                [&runtime_state, carriage_origin_angle_interface](CallbackAction&) {
                    runtime_state.carriage_calibration_origin_samples.push_back(
                        *carriage_origin_angle_interface);
                    return ActionStatus::SUCCESS;
                }));
        then(std::make_shared<DelayAction>("delay", 100));
        then(make_return_action());
        then(std::make_shared<DelayAction>("delay", 100));
    }
};

class CarriageInitFinalizeTask : public Task {
public:
    CarriageInitFinalizeTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings, ManagerRuntimeState& runtime_state)
        : Task("carriage_init_finalize", "丝杆初始化平均值收敛") {
        auto carriage_origin_angle_interface = &output.carriage_origin_angle;

        auto make_return_close_loop_action = [&input, &output, &settings]() {
            return std::make_shared<CarriageAngleCloseLoopAction>(
                "carriage_angle_close_loop", output.carriage_command,
                output.carriage_target_velocity, output.carriage_target_angle, input.carriage_angle,
                input.carriage_origin_angle, rmcs_msgs::DartMechanismCommand::UP,
                settings.carriage_up_setting_velocity, settings.carriage_down_travel_angle,
                settings.carriage_angle_allowable_error, settings.carriage_min_run_ticks,
                settings.carriage_timeout_ticks);
        };

        then(
            std::make_shared<CallbackAction>(
                "carriage_origin_average_finalize",
                [&runtime_state, carriage_origin_angle_interface](CallbackAction& action) {
                    constexpr std::size_t kCalibrationPassCount = 3;
                    if (runtime_state.carriage_calibration_origin_samples.size()
                        != kCalibrationPassCount) {
                        return action.fail_with(ActionFailureReason::INVALID_INPUT);
                    }
                    const double average_origin_angle =
                        std::accumulate(
                            runtime_state.carriage_calibration_origin_samples.begin(),
                            runtime_state.carriage_calibration_origin_samples.end(), 0.0)
                        / static_cast<double>(kCalibrationPassCount);
                    *carriage_origin_angle_interface = average_origin_angle;
                    runtime_state.carriage_power_cycle_origin_angle = average_origin_angle;
                    runtime_state.carriage_calibration_origin_samples.clear();
                    return ActionStatus::SUCCESS;
                }));
        then(std::make_shared<DelayAction>("delay", 100));
        then(make_return_close_loop_action());
    }
};

} // namespace rmcs_dart_guidance::manager
