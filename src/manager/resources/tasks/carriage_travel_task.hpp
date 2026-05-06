#pragma once

#include <memory>

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/carriage_control_action.hpp"

namespace rmcs_dart_guidance::manager {

class CarriageTravelTask : public Task {
public:
    CarriageTravelTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings, const ManagerRuntimeState& runtime_state)
        : Task("carriage_travel", "发射滑台运动") {
        then(
            std::make_shared<CarriageTravelAction>(
                "carriage_travel", output.carriage_command, output.carriage_target_velocity,
                input.carriage_angle, input.carriage_velocity, input.carriage_torque,
                rmcs_msgs::DartMechanismCommand::DOWN, settings.carriage_down_setting_velocity,
                settings.carriage_travel_distance, 20000,
                runtime_state.carriage_init_reference_angle));
    }
};

} // namespace rmcs_dart_guidance::manager
