#pragma once

#include "manager/runtime/manager_types.hpp"
#include "manager/runtime/task.hpp"
#include <rmcs_dart_guidance/action/configuration_guard_action.hpp>
#include <rmcs_dart_guidance/action/trigger_command_action.hpp>
#include <rmcs_dart_guidance/resource/mechanism_resources.hpp>

#include <memory>

#include <rmcs_dart_guidance/msg/trigger_command.hpp>

namespace rmcs_dart_guidance::manager {

class DartCarriageCalibrateTask : public Task {
public:
    DartCarriageCalibrateTask(MechanismResources& resources, const ManagerSettings& settings)
        : Task("dart-carriage-calibrate", "滑台标定") {
        using rmcs_dart_guidance::msg::TriggerCommand;
        then(
            std::make_shared<TriggerCommandAction>(
                "trigger_lock", resources.trigger, TriggerCommand::TRIGGER_LOCK,
                kServoTimeoutTicks));
        then(
            std::make_shared<ConfigurationGuardAction>(
                "launch_carriage_positions_configured",
                settings.launch_carriage_positions_configured));
        then(
            std::make_shared<TriggerCommandAction>(
                "carriage_calibrate", resources.trigger, TriggerCommand::CARRIAGE_CALIBRATE,
                kMechanismTimeoutTicks));
        then(
            std::make_shared<TriggerCommandAction>(
                "carriage_goto_launch_1", resources.trigger, TriggerCommand::CARRIAGE_GOTO,
                kMechanismTimeoutTicks, settings.launch_carriage_positions[0]));
        then(
            std::make_shared<TriggerCommandAction>(
                "trigger_free", resources.trigger, TriggerCommand::TRIGGER_FREE,
                kServoTimeoutTicks));
    }

private:
    static constexpr uint64_t kServoTimeoutTicks = 1000;
    static constexpr uint64_t kMechanismTimeoutTicks = 100000;
};

} // namespace rmcs_dart_guidance::manager
