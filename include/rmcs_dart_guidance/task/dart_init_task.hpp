#pragma once

#include "manager/runtime/action_set.hpp"
#include "manager/runtime/action_sequence.hpp"
#include "manager/runtime/manager_types.hpp"
#include "manager/runtime/task.hpp"
#include <rmcs_dart_guidance/action/belt_command_action.hpp>
#include <rmcs_dart_guidance/action/configuration_guard_action.hpp>
#include <rmcs_dart_guidance/action/filling_command_action.hpp>
#include <rmcs_dart_guidance/action/trigger_command_action.hpp>
#include <rmcs_dart_guidance/resource/mechanism_resources.hpp>

#include <limits>
#include <memory>

#include <rmcs_dart_guidance/msg/belt_command.hpp>
#include <rmcs_dart_guidance/msg/filling_command.hpp>
#include <rmcs_dart_guidance/msg/trigger_command.hpp>

namespace rmcs_dart_guidance::manager {

class DartInitTask : public Task {
public:
    explicit DartInitTask(MechanismResources& resources, const ManagerSettings& settings)
        : Task("dart-init", "初始化") {
        using rmcs_dart_guidance::msg::BeltCommand;
        using rmcs_dart_guidance::msg::FillingCommand;
        using rmcs_dart_guidance::msg::TriggerCommand;

        auto init = std::make_shared<ActionSet>("dart_init_all");
        init->also(
            std::make_shared<BeltCommandAction>(
                "belt_init", resources.belt, BeltCommand::INIT, kMechanismTimeoutTicks));
        init->also(
            std::make_shared<FillingCommandAction>(
                "filling_lift_up", resources.filling, FillingCommand::LIFT_UP,
                kMechanismTimeoutTicks));

        auto carriage_seq = std::make_shared<ActionSequence>("carriage_calibrate_seq");
        carriage_seq->then(
            std::make_shared<TriggerCommandAction>(
                "trigger_lock", resources.trigger, TriggerCommand::TRIGGER_LOCK,
                kServoTimeoutTicks));
        carriage_seq->then(
            std::make_shared<ConfigurationGuardAction>(
                "launch_carriage_positions_configured",
                settings.launch_carriage_positions_configured));
        carriage_seq->then(
            std::make_shared<TriggerCommandAction>(
                "carriage_calibrate", resources.trigger, TriggerCommand::CARRIAGE_CALIBRATE,
                kCarriageCalibrateTimeoutTicks));
        carriage_seq->then(
            std::make_shared<TriggerCommandAction>(
                "carriage_goto_launch_1", resources.trigger, TriggerCommand::CARRIAGE_GOTO,
                kCarriageCalibrateTimeoutTicks, settings.launch_carriage_positions[0]));
        carriage_seq->then(
            std::make_shared<TriggerCommandAction>(
                "trigger_free", resources.trigger, TriggerCommand::TRIGGER_FREE,
                kServoTimeoutTicks));
        init->also(carriage_seq);

        then(init);
    }

private:
    static constexpr uint64_t kMechanismTimeoutTicks = 20000;
    static constexpr uint64_t kCarriageCalibrateTimeoutTicks = 100000;
    static constexpr uint64_t kServoTimeoutTicks = 1000;
};

} // namespace rmcs_dart_guidance::manager
