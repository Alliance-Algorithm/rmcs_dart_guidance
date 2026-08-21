#pragma once

#include "manager/runtime/manager_types.hpp"
#include "manager/runtime/task.hpp"
#include "rmcs_dart_guidance/action/delay_action.hpp"
#include <rmcs_dart_guidance/action/configuration_guard_action.hpp>
#include <rmcs_dart_guidance/action/fire_count_increment_action.hpp>
#include <rmcs_dart_guidance/action/trigger_command_action.hpp>
#include <rmcs_dart_guidance/resource/mechanism_resources.hpp>

#include <algorithm>
#include <cstddef>
#include <memory>

#include <rmcs_dart_guidance/msg/trigger_command.hpp>

namespace rmcs_dart_guidance::manager {

class DartFireTask : public Task {
public:
    DartFireTask(
        MechanismResources& resources, ManagerRuntimeState& runtime_state,
        const ManagerSettings& settings)
        : Task("dart-fire", "发射") {
        using rmcs_dart_guidance::msg::TriggerCommand;

        const std::size_t goto_index =
            std::min<std::size_t>(static_cast<std::size_t>(runtime_state.fire_count) + 1, 3);

        then(std::make_shared<DelayAction>("delay", 1000));

        then(
            std::make_shared<ConfigurationGuardAction>(
                "launch_carriage_positions_configured",
                settings.launch_carriage_positions_configured));
        then(
            std::make_shared<TriggerCommandAction>(
                "trigger_free", resources.trigger, TriggerCommand::TRIGGER_FREE,
                kServoTimeoutTicks));

        then(std::make_shared<DelayAction>("delay", 500));

        then(
            std::make_shared<TriggerCommandAction>(
                "carriage_goto_next_launch", resources.trigger, TriggerCommand::CARRIAGE_GOTO,
                kMechanismTimeoutTicks, settings.launch_carriage_positions[goto_index]));
        then(
            std::make_shared<FireCountIncrementAction>(
                "fire_count_increment", runtime_state.fire_count));
    }

private:
    static constexpr uint64_t kServoTimeoutTicks = 1000;
    static constexpr uint64_t kMechanismTimeoutTicks = 100000;
};

} // namespace rmcs_dart_guidance::manager
