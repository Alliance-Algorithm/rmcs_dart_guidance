#pragma once

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/filling_lift_action.hpp"

#include <memory>

namespace rmcs_dart_guidance::manager {

class FillingLiftUpTask : public Task {
public:
    FillingLiftUpTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings)
        : Task("filling_lift_up", "填装机构上行") {

        then(
            std::make_shared<FillingLiftAction>(
                "filling_lift_up", output.lifting_command, output.lift_target_velocity,
                output.lift_exit_mode, input.lift_left_velocity, input.lift_left_torque,
                input.lift_right_velocity, input.lift_right_torque,
                rmcs_msgs::DartMechanismCommand::UP, settings.lift_target_velocity,
                rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY, settings.lift_stall_velocity_threshold,
                settings.lift_stall_torque_threshold, settings.lift_stall_confirm_ticks, 20000));
    }
};

class FillingLiftDownTask : public Task {
public:
    FillingLiftDownTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings)
        : Task("filling_lift_down", "填装机构下行") {

        then(
            std::make_shared<FillingLiftAction>(
                "filling_lift_down", output.lifting_command, output.lift_target_velocity,
                output.lift_exit_mode, input.lift_left_velocity, input.lift_left_torque,
                input.lift_right_velocity, input.lift_right_torque,
                rmcs_msgs::DartMechanismCommand::DOWN, settings.lift_target_velocity,
                rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY, settings.lift_stall_velocity_threshold,
                settings.lift_stall_torque_threshold, settings.lift_stall_confirm_ticks, 20000));
    }
};

} // namespace rmcs_dart_guidance::manager
