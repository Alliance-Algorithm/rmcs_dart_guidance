#pragma once

#include "manager/runtime/action_set.hpp"
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

class DartLaunchPrepareTask : public Task {
public:
    DartLaunchPrepareTask(MechanismResources& resources, const ManagerRuntimeState& runtime_state)
        : Task("dart-launch-prepare", "发射准备") {
        using rmcs_dart_guidance::msg::BeltCommand;
        using rmcs_dart_guidance::msg::FillingCommand;
        using rmcs_dart_guidance::msg::TriggerCommand;

        if (runtime_state.fire_count == 0) {
            then(
                std::make_shared<BeltCommandAction>(
                    "belt_down_slow", resources.belt, BeltCommand::DOWN_SLOW,
                    kMechanismTimeoutTicks));
            then(
                std::make_shared<TriggerCommandAction>(
                    "trigger_lock", resources.trigger, TriggerCommand::TRIGGER_LOCK,
                    kServoTimeoutTicks));
            then(
                std::make_shared<BeltCommandAction>(
                    "belt_up_soft", resources.belt, BeltCommand::UP_SOFT,
                    kMechanismTimeoutTicks));
            return;
        }

        then(
            std::make_shared<BeltCommandAction>(
                "belt_down_fast", resources.belt, BeltCommand::DOWN_FAST,
                kMechanismTimeoutTicks));
        then(
            std::make_shared<FillingCommandAction>(
                "filling_lift_down", resources.filling, FillingCommand::LIFT_DOWN,
                kMechanismTimeoutTicks));
        then(
            std::make_shared<BeltCommandAction>(
                "belt_up_soft_part", resources.belt, BeltCommand::UP_SOFT_PART,
                kMechanismTimeoutTicks));
        then(
            std::make_shared<FillingCommandAction>(
                "filling_lift_up", resources.filling, FillingCommand::LIFT_UP,
                kMechanismTimeoutTicks));
        then(
            std::make_shared<BeltCommandAction>(
                "belt_down_slow_part", resources.belt, BeltCommand::DOWN_SLOW_PART,
                kMechanismTimeoutTicks));
        then(
            std::make_shared<TriggerCommandAction>(
                "trigger_lock", resources.trigger, TriggerCommand::TRIGGER_LOCK,
                kServoTimeoutTicks));

        auto preload = std::make_shared<ActionSet>("belt_up_soft_and_limit_pulse");
        preload->also(
            std::make_shared<BeltCommandAction>(
                "belt_up_soft", resources.belt, BeltCommand::UP_SOFT,
                kMechanismTimeoutTicks));
        preload->also(
            std::make_shared<FillingCommandAction>(
                "filling_limit_pulse", resources.filling, FillingCommand::LIMIT_PULSE_FILL,
                kLimitPulseTimeoutTicks));
        then(preload);
    }

private:
    static constexpr uint64_t kMechanismTimeoutTicks = 5000;
    static constexpr uint64_t kServoTimeoutTicks = 200;
    static constexpr uint64_t kLimitPulseTimeoutTicks = 500;
};

} // namespace rmcs_dart_guidance::manager
