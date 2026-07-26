#pragma once

#include "manager/runtime/task.hpp"
#include <rmcs_dart_guidance/action/belt_command_action.hpp>
#include <rmcs_dart_guidance/action/trigger_command_action.hpp>
#include <rmcs_dart_guidance/resource/mechanism_resources.hpp>

#include <memory>

#include <rmcs_dart_guidance/msg/belt_command.hpp>
#include <rmcs_dart_guidance/msg/trigger_command.hpp>

namespace rmcs_dart_guidance::manager {

class DartLaunchCancelTask : public Task {
public:
    explicit DartLaunchCancelTask(MechanismResources& resources)
        : Task("dart-launch-cancel", "取消发射") {
        using rmcs_dart_guidance::msg::BeltCommand;
        using rmcs_dart_guidance::msg::TriggerCommand;

        then(
            std::make_shared<BeltCommandAction>(
                "belt_down_fast", resources.belt, BeltCommand::DOWN_FAST, kMechanismTimeoutTicks));
        then(
            std::make_shared<TriggerCommandAction>(
                "trigger_free", resources.trigger, TriggerCommand::TRIGGER_FREE,
                kServoTimeoutTicks));
        then(
            std::make_shared<BeltCommandAction>(
                "belt_up_hard", resources.belt, BeltCommand::UP_HARD, kMechanismTimeoutTicks));
    }

private:
    static constexpr uint64_t kMechanismTimeoutTicks = 20000;
    static constexpr uint64_t kServoTimeoutTicks = 1000;
};

} // namespace rmcs_dart_guidance::manager
