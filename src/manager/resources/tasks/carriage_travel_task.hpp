#pragma once

#include <memory>
#include <string>
#include <utility>

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/carriage_control_action.hpp"

namespace rmcs_dart_guidance::manager {

class CarriageTravelTask : public Task {
public:
    CarriageTravelTask(
        std::string task_name, std::string task_description, const ManagerInputContext& input,
        ManagerOutputContext& output, rmcs_msgs::DartMechanismCommand command,
        double velocity_setting, double travel_distance, const ManagerSettings& settings,
        const ManagerRuntimeState& runtime_state,
        CarriageTravelAction::TravelReferenceMode travel_reference_mode)
        : Task(std::move(task_name), std::move(task_description)) {
        then(
            std::make_shared<CarriageTravelAction>(
                "carriage_travel", output.carriage_command, output.carriage_target_velocity,
                input.carriage_angle, input.carriage_velocity, input.carriage_torque, command,
                velocity_setting, travel_distance, settings.carriage_stall_velocity_threshold,
                settings.carriage_stall_torque_threshold, settings.carriage_stall_confirm_ticks,
                20000, runtime_state.carriage_power_cycle_origin_angle, travel_reference_mode));
    }

    CarriageTravelTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings, const ManagerRuntimeState& runtime_state)
        : CarriageTravelTask(
              "carriage_travel", "发射滑台运动", input, output, rmcs_msgs::DartMechanismCommand::DOWN,
              settings.carriage_down_setting_velocity, settings.carriage_travel_distance, settings,
              runtime_state, CarriageTravelAction::TravelReferenceMode::ORIGIN_ABSOLUTE) {}
};

} // namespace rmcs_dart_guidance::manager
