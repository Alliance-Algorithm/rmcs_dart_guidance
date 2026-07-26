#pragma once

#include "manager/runtime/manager_types.hpp"
#include "manager/runtime/task.hpp"
#include <rmcs_dart_guidance/action/belt_command_action.hpp>
#include <rmcs_dart_guidance/action/filling_command_action.hpp>
#include <rmcs_dart_guidance/action/trigger_command_action.hpp>
#include <rmcs_dart_guidance/resource/mechanism_resources.hpp>

#include <memory>

#include <rmcs_dart_guidance/msg/belt_command.hpp>
#include <rmcs_dart_guidance/msg/filling_command.hpp>
#include <rmcs_dart_guidance/msg/trigger_command.hpp>

namespace rmcs_dart_guidance::manager {

class LaunchPreparationTask : public Task {
public:
    LaunchPreparationTask(MechanismResources& resources, const ManagerRuntimeState& runtime_state)
        : Task("launch_prepare", "发射准备") {
        using rmcs_dart_guidance::msg::BeltCommand;
        using rmcs_dart_guidance::msg::FillingCommand;
        using rmcs_dart_guidance::msg::TriggerCommand;

        const auto belt_command_down =
            runtime_state.fire_count == 0 ? BeltCommand::DOWN_SLOW : BeltCommand::DOWN_FAST;

        then(
            std::make_shared<BeltCommandAction>(
                "belt_down_1", resources.belt, belt_command_down, 200));

        then(
            std::make_shared<TimedBeltCommandAction>(
                "belt_down_hold", resources.belt, BeltCommand::BRAKE, 200));
        then(
            std::make_shared<TriggerCommandAction>(
                "trigger_lock", resources.trigger, TriggerCommand::TRIGGER_LOCK, 200));

        if (runtime_state.fire_count > 0) {
            then(
                std::make_shared<FillingCommandAction>(
                    "filling_lift_down", resources.filling, FillingCommand::LIFT_DOWN, 200));
        }

        then(
            std::make_shared<BeltCommandAction>(
                "belt_up_soft", resources.belt, BeltCommand::UP_SOFT, 200));
        then(
            std::make_shared<BeltCommandAction>(
                "belt_up_hard", resources.belt, BeltCommand::UP_HARD, 200));
    }
};

} // namespace rmcs_dart_guidance::manager
