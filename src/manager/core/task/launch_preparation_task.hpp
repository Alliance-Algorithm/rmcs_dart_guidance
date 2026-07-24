#pragma once

#include "manager/core/action/action_timeouts.hpp"
#include "manager/core/action/mechanism_command_action.hpp"
#include "manager/core/runtime/manager_types.hpp"
#include "manager/core/runtime/task.hpp"
#include "manager/resources/mechanism_resources.hpp"

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

        const auto first_down =
            runtime_state.fire_count == 0 ? BeltCommand::DOWN_SLOW : BeltCommand::DOWN_FAST;

        then(
            std::make_shared<BeltCommandAction>(
                "belt_down_1", resources.belt, first_down, kTimeoutBeltDown));
        then(
            std::make_shared<BeltCommandAction>(
                "belt_down_hold", resources.belt, BeltCommand::DOWN_HOLD, kTimeoutBeltDownHold));
        then(
            std::make_shared<TriggerCommandAction>(
                "trigger_lock", resources.trigger, TriggerCommand::SERVO_LOCK,
                kTimeoutTriggerServo));

        if (runtime_state.fire_count > 0) {
            then(
                std::make_shared<FillingCommandAction>(
                    "filling_lift_down", resources.filling, FillingCommand::LIFT_DOWN,
                    kTimeoutFillingLift));
        }

        then(
            std::make_shared<BeltCommandAction>(
                "belt_up_soft", resources.belt, BeltCommand::UP_SOFT, kTimeoutBeltUpSoft));
        then(
            std::make_shared<BeltCommandAction>(
                "belt_up_hard", resources.belt, BeltCommand::UP_HARD, kTimeoutBeltUpHard));
        then(
            std::make_shared<BeltCommandAction>(
                "belt_up_stall", resources.belt, BeltCommand::UP_STALL, kTimeoutBeltUpStall));
    }
};

} // namespace rmcs_dart_guidance::manager
