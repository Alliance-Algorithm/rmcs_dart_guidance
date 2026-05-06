#pragma once

#include <cstdlib>
#include <memory>

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/carriage_control_action.hpp"

namespace rmcs_dart_guidance::manager {

class CarriageInitTask : public Task {
public:
    CarriageInitTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings, ManagerRuntimeState& runtime_state)
        : Task("carriage_init", "丝杆低速堵转初始化") {

        then(
            std::make_shared<CarriageControlAction>(
                "carriage_init", output.carriage_command, output.carriage_target_velocity,
                input.carriage_velocity, input.carriage_torque, rmcs_msgs::DartMechanismCommand::UP,
                settings.carriage_up_setting_velocity, settings.carriage_stall_velocity_threshold,
                settings.carriage_stall_torque_threshold, 100, 20000, &input.carriage_angle,
                &runtime_state.carriage_init_reference_angle));
    }
};

} // namespace rmcs_dart_guidance::manager
