#pragma once

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

class CancelLaunchTask : public Task {
public:
    explicit CancelLaunchTask(MechanismResources& resources)
        : Task("launch_cancel", "取消发射") {
        using rmcs_dart_guidance::msg::BeltCommand;
        using rmcs_dart_guidance::msg::FillingCommand;
        using rmcs_dart_guidance::msg::TriggerCommand;

        then(
            std::make_shared<BeltCommandAction>(
                "belt_down_fast", resources.belt, BeltCommand::DOWN_FAST, 200));
        then(
            std::make_shared<BeltCommandAction>(
                "belt_down_hold", resources.belt, BeltCommand::STOP, 200));
        then(
            std::make_shared<TriggerCommandAction>(
                "trigger_free", resources.trigger, TriggerCommand::SERVO_FREE, 200));
        then(
            std::make_shared<BeltCommandAction>(
                "belt_up_soft", resources.belt, BeltCommand::UP_SOFT, 200));
        then(
            std::make_shared<BeltCommandAction>(
                "belt_up_hard", resources.belt, BeltCommand::UP_HARD, 200));
        then(
            std::make_shared<BeltCommandAction>(
                "belt_up_stall", resources.belt, BeltCommand::UP_STALL, 500));
        then(
            std::make_shared<FillingCommandAction>(
                "filling_lift_up", resources.filling, FillingCommand::LIFT_UP, 200));
    }
};

} // namespace rmcs_dart_guidance::manager
