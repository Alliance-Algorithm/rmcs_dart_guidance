#pragma once

#include "manager/core/action/action_timeouts.hpp"
#include "manager/core/action/mechanism_command_action.hpp"
#include "manager/core/runtime/task.hpp"
#include "manager/resources/mechanism_resources.hpp"

#include <memory>

#include <rmcs_dart_guidance/msg/belt_command.hpp>
#include <rmcs_dart_guidance/msg/filling_command.hpp>
#include <rmcs_dart_guidance/msg/trigger_command.hpp>

namespace rmcs_dart_guidance::manager {

class CancelLaunchTask : public Task {
public:
    explicit CancelLaunchTask(MechanismResources& resources)
        : Task("launch_cancel", "取消发射") {
        using rmcs_dart_guidance::msg::BeltCommand;
        using rmcs_dart_guidance::msg::FillingCommand;
        using rmcs_dart_guidance::msg::TriggerCommand;

        then(
            std::make_shared<BeltCommandAction>(
                "belt_down_fast", resources.belt, BeltCommand::DOWN_FAST, kTimeoutBeltDown));
        then(
            std::make_shared<BeltCommandAction>(
                "belt_down_hold", resources.belt, BeltCommand::DOWN_HOLD, kTimeoutBeltDownHold));
        then(
            std::make_shared<TriggerCommandAction>(
                "trigger_free", resources.trigger, TriggerCommand::SERVO_FREE,
                kTimeoutTriggerServo));
        then(
            std::make_shared<BeltCommandAction>(
                "belt_up_soft", resources.belt, BeltCommand::UP_SOFT, kTimeoutBeltUpSoft));
        then(
            std::make_shared<BeltCommandAction>(
                "belt_up_hard", resources.belt, BeltCommand::UP_HARD, kTimeoutBeltUpHard));
        then(
            std::make_shared<BeltCommandAction>(
                "belt_up_stall", resources.belt, BeltCommand::UP_STALL, kTimeoutBeltUpStall));
        then(
            std::make_shared<FillingCommandAction>(
                "filling_lift_up", resources.filling, FillingCommand::LIFT_UP,
                kTimeoutFillingLift));
    }
};

} // namespace rmcs_dart_guidance::manager
