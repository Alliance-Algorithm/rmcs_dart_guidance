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

            auto trigger_lock =
                std::make_shared<ActionSet>("trigger_lock", ActionSet::Policy::ANY_SUCCESS);
            trigger_lock->also(
                std::make_shared<BeltCommandAction>(
                    "belt_brake", resources.belt, BeltCommand::BRAKE, kMechanismTimeoutTicks));
            trigger_lock->also(
                std::make_shared<TriggerCommandAction>(
                    "trigger_lock", resources.trigger, TriggerCommand::TRIGGER_LOCK,
                    kServoTimeoutTicks));
            then(trigger_lock);

            then(
                std::make_shared<BeltCommandAction>(
                    "belt_up_soft", resources.belt, BeltCommand::UP_SOFT, kMechanismTimeoutTicks));
            return;
        }

        then(
            std::make_shared<BeltCommandAction>(
                "belt_down_fast", resources.belt, BeltCommand::DOWN_FAST, kMechanismTimeoutTicks));

        auto dart_filling =
            std::make_shared<ActionSet>("dart_filling", ActionSet::Policy::ANY_SUCCESS);
        dart_filling->also(
            std::make_shared<BeltCommandAction>(
                "belt_brake", resources.belt, BeltCommand::BRAKE, kMechanismTimeoutTicks));
        dart_filling->also(
            std::make_shared<FillingCommandAction>(
                "filling_lift_down", resources.filling, FillingCommand::LIFT_DOWN,
                kMechanismTimeoutTicks));
        then(dart_filling);

        then(
            std::make_shared<BeltCommandAction>(
                "belt_up_soft_part", resources.belt, BeltCommand::UP_SOFT_PART,
                kMechanismTimeoutTicks));

        auto belt_wait_lift_up =
            std::make_shared<ActionSet>("belt_wait_lift_up", ActionSet::Policy::ANY_SUCCESS);
        belt_wait_lift_up->also(
            std::make_shared<BeltCommandAction>(
                "belt_brake", resources.belt, BeltCommand::BRAKE, kMechanismTimeoutTicks));
        belt_wait_lift_up->also(
            std::make_shared<FillingCommandAction>(
                "filling_lift_up", resources.filling, FillingCommand::LIFT_UP,
                kMechanismTimeoutTicks));
        then(belt_wait_lift_up);

        then(
            std::make_shared<BeltCommandAction>(
                "belt_down_slow_part", resources.belt, BeltCommand::DOWN_SLOW_PART,
                kMechanismTimeoutTicks));

        auto trigger_lock =
            std::make_shared<ActionSet>("trigger_lock", ActionSet::Policy::ANY_SUCCESS);
        trigger_lock->also(
            std::make_shared<BeltCommandAction>(
                "belt_brake", resources.belt, BeltCommand::BRAKE, kMechanismTimeoutTicks));
        trigger_lock->also(
            std::make_shared<TriggerCommandAction>(
                "trigger_lock", resources.trigger, TriggerCommand::TRIGGER_LOCK,
                kServoTimeoutTicks));
        then(trigger_lock);

        auto preload = std::make_shared<ActionSet>("belt_up_soft_and_limit_pulse");
        preload->also(
            std::make_shared<BeltCommandAction>(
                "belt_up_soft", resources.belt, BeltCommand::UP_SOFT, kMechanismTimeoutTicks));
        preload->also(
            std::make_shared<FillingCommandAction>(
                "filling_limit_pulse", resources.filling, FillingCommand::LIMIT_PULSE_FILL,
                kLimitPulseTimeoutTicks));
        then(preload);
    }

private:
    static constexpr uint64_t kMechanismTimeoutTicks = 20000;
    static constexpr uint64_t kServoTimeoutTicks = 1000;
    static constexpr uint64_t kLimitPulseTimeoutTicks = 1000;
};

} // namespace rmcs_dart_guidance::manager
