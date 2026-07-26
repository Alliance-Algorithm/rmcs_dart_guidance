#pragma once

#include "manager/runtime/manager_types.hpp"
#include "manager/runtime/task.hpp"
#include <rmcs_dart_guidance/action/fire_count_increment_action.hpp>
#include <rmcs_dart_guidance/action/trigger_command_action.hpp>
#include <rmcs_dart_guidance/resource/mechanism_resources.hpp>

#include <memory>

#include <rmcs_dart_guidance/msg/trigger_command.hpp>

namespace rmcs_dart_guidance::manager {

class DartFireTask : public Task {
public:
    DartFireTask(MechanismResources& resources, ManagerRuntimeState& runtime_state)
        : Task("dart-fire", "发射") {
        using rmcs_dart_guidance::msg::TriggerCommand;

        then(
            std::make_shared<TriggerCommandAction>(
                "trigger_free", resources.trigger, TriggerCommand::TRIGGER_FREE,
                kServoTimeoutTicks));
        then(
            std::make_shared<FireCountIncrementAction>(
                "fire_count_increment", runtime_state.fire_count));
    }

private:
    static constexpr uint64_t kServoTimeoutTicks = 200;
};

} // namespace rmcs_dart_guidance::manager
