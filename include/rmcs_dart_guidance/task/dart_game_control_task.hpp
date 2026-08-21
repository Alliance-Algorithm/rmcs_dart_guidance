#pragma once

#include "manager/runtime/action_set.hpp"
#include "manager/runtime/manager_types.hpp"
#include "manager/runtime/task.hpp"
#include <rmcs_dart_guidance/action/belt_command_action.hpp>
#include <rmcs_dart_guidance/action/configuration_guard_action.hpp>
#include <rmcs_dart_guidance/action/delay_action.hpp>
#include <rmcs_dart_guidance/action/filling_command_action.hpp>
#include <rmcs_dart_guidance/action/fire_count_increment_action.hpp>
#include <rmcs_dart_guidance/action/trigger_command_action.hpp>
#include <rmcs_dart_guidance/action/yaw_issue_command_action.hpp>
#include <rmcs_dart_guidance/resource/mechanism_resources.hpp>

#include <algorithm>
#include <memory>

#include <rmcs_dart_guidance/msg/belt_command.hpp>
#include <rmcs_dart_guidance/msg/filling_command.hpp>
#include <rmcs_dart_guidance/msg/trigger_command.hpp>
#include <rmcs_dart_guidance/msg/yaw_command.hpp>

namespace rmcs_dart_guidance::manager {

class DartGameControlTask : public Task {
public:
    DartGameControlTask(
        MechanismResources& resources, ManagerRuntimeState& runtime_state,
        const ManagerSettings& settings)
        : Task("dart-game-control", "比赛发射") {
        const uint32_t initial_fire_count = runtime_state.fire_count;

        build_launch_prepare(resources, settings, initial_fire_count == 0, initial_fire_count, 1);
        build_fire(resources, runtime_state, settings, initial_fire_count + 1, 1);
        build_launch_prepare(resources, settings, false, initial_fire_count + 1, 2);
        build_fire(resources, runtime_state, settings, initial_fire_count + 2, 2);
    }

private:
    void build_launch_prepare(
        MechanismResources& resources, const ManagerSettings& settings, bool is_first,
        uint32_t fire_count, int suffix) {
        using rmcs_dart_guidance::msg::BeltCommand;
        using rmcs_dart_guidance::msg::FillingCommand;
        using rmcs_dart_guidance::msg::TriggerCommand;
        using rmcs_dart_guidance::msg::YawCommand;

        const std::size_t setpoint_index =
            std::min<std::size_t>(static_cast<std::size_t>(fire_count), 3);

        then(std::make_shared<ConfigurationGuardAction>(
            "vision_aim_target_setpoints_configured_" + std::to_string(suffix),
            settings.vision_aim_target_setpoints_configured));
        then(std::make_shared<YawIssueCommandAction>(
            "yaw_vision_aim_begin_" + std::to_string(suffix), resources.yaw,
            YawCommand::VISION_AIM, settings.vision_aim_target_setpoints[setpoint_index]));

        if (is_first) {
            then(std::make_shared<BeltCommandAction>(
                "belt_down_slow_" + std::to_string(suffix), resources.belt,
                BeltCommand::DOWN_SLOW, kMechanismTimeoutTicks));
            auto trigger_lock = std::make_shared<ActionSet>(
                "trigger_lock_" + std::to_string(suffix), ActionSet::Policy::ANY_SUCCESS);
            trigger_lock->also(std::make_shared<BeltCommandAction>(
                "belt_brake_a" + std::to_string(suffix), resources.belt, BeltCommand::BRAKE,
                kMechanismTimeoutTicks));
            trigger_lock->also(std::make_shared<TriggerCommandAction>(
                "trigger_lock_" + std::to_string(suffix), resources.trigger,
                TriggerCommand::TRIGGER_LOCK, kServoTimeoutTicks));
            then(trigger_lock);
            then(std::make_shared<BeltCommandAction>(
                "belt_up_soft_" + std::to_string(suffix), resources.belt, BeltCommand::UP_SOFT,
                kMechanismTimeoutTicks));
        } else {
            then(std::make_shared<BeltCommandAction>(
                "belt_down_fast_" + std::to_string(suffix), resources.belt,
                BeltCommand::DOWN_FAST, kMechanismTimeoutTicks));

            auto filling_down = std::make_shared<ActionSet>(
                "filling_down_" + std::to_string(suffix), ActionSet::Policy::ANY_SUCCESS);
            filling_down->also(std::make_shared<BeltCommandAction>(
                "belt_brake_b" + std::to_string(suffix), resources.belt, BeltCommand::BRAKE,
                kMechanismTimeoutTicks));
            filling_down->also(std::make_shared<FillingCommandAction>(
                "filling_lift_down_" + std::to_string(suffix), resources.filling,
                FillingCommand::LIFT_DOWN, kMechanismTimeoutTicks));
            then(filling_down);

            then(std::make_shared<BeltCommandAction>(
                "belt_up_soft_part_" + std::to_string(suffix), resources.belt,
                BeltCommand::UP_SOFT_PART, kMechanismTimeoutTicks));

            auto filling_up = std::make_shared<ActionSet>(
                "filling_up_" + std::to_string(suffix), ActionSet::Policy::ANY_SUCCESS);
            filling_up->also(std::make_shared<BeltCommandAction>(
                "belt_brake_c" + std::to_string(suffix), resources.belt, BeltCommand::BRAKE,
                kMechanismTimeoutTicks));
            filling_up->also(std::make_shared<FillingCommandAction>(
                "filling_lift_up_" + std::to_string(suffix), resources.filling,
                FillingCommand::LIFT_UP, kMechanismTimeoutTicks));
            then(filling_up);

            then(std::make_shared<BeltCommandAction>(
                "belt_down_slow_part_" + std::to_string(suffix), resources.belt,
                BeltCommand::DOWN_SLOW_PART, kMechanismTimeoutTicks));

            auto trigger_lock = std::make_shared<ActionSet>(
                "trigger_lock_" + std::to_string(suffix), ActionSet::Policy::ANY_SUCCESS);
            trigger_lock->also(std::make_shared<BeltCommandAction>(
                "belt_brake_d" + std::to_string(suffix), resources.belt, BeltCommand::BRAKE,
                kMechanismTimeoutTicks));
            trigger_lock->also(std::make_shared<TriggerCommandAction>(
                "trigger_lock_" + std::to_string(suffix), resources.trigger,
                TriggerCommand::TRIGGER_LOCK, kServoTimeoutTicks));
            then(trigger_lock);

            auto preload = std::make_shared<ActionSet>(
                "preload_" + std::to_string(suffix));
            preload->also(std::make_shared<BeltCommandAction>(
                "belt_up_soft_" + std::to_string(suffix), resources.belt, BeltCommand::UP_SOFT,
                kMechanismTimeoutTicks));
            preload->also(std::make_shared<FillingCommandAction>(
                "filling_limit_pulse_" + std::to_string(suffix), resources.filling,
                FillingCommand::LIMIT_PULSE_FILL, kLimitPulseTimeoutTicks));
            then(preload);
        }

        then(std::make_shared<YawIssueCommandAction>(
            "yaw_vision_aim_end_" + std::to_string(suffix), resources.yaw, YawCommand::IDLE));
    }

    void build_fire(
        MechanismResources& resources, ManagerRuntimeState& runtime_state,
        const ManagerSettings& settings, uint32_t goto_raw, int suffix) {
        using rmcs_dart_guidance::msg::TriggerCommand;

        const std::size_t goto_index =
            std::min<std::size_t>(static_cast<std::size_t>(goto_raw), 3);

        then(std::make_shared<DelayAction>("delay_a" + std::to_string(suffix), 1000));
        then(std::make_shared<ConfigurationGuardAction>(
            "launch_carriage_positions_configured_" + std::to_string(suffix),
            settings.launch_carriage_positions_configured));
        then(std::make_shared<TriggerCommandAction>(
            "trigger_free_" + std::to_string(suffix), resources.trigger,
            TriggerCommand::TRIGGER_FREE, kServoTimeoutTicks));
        then(std::make_shared<DelayAction>("delay_b" + std::to_string(suffix), 500));
        then(std::make_shared<TriggerCommandAction>(
            "carriage_goto_" + std::to_string(suffix), resources.trigger,
            TriggerCommand::CARRIAGE_GOTO, kMechanismTimeoutTicks,
            settings.launch_carriage_positions[goto_index]));
        then(std::make_shared<FireCountIncrementAction>(
            "fire_count_increment_" + std::to_string(suffix), runtime_state.fire_count));
    }

    static constexpr uint64_t kMechanismTimeoutTicks = 20000;
    static constexpr uint64_t kServoTimeoutTicks = 1000;
    static constexpr uint64_t kLimitPulseTimeoutTicks = 1000;
};

} // namespace rmcs_dart_guidance::manager
