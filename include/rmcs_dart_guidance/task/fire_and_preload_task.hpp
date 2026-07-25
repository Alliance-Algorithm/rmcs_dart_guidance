#pragma once

#include <rmcs_dart_guidance/action/delay_action.hpp>
#include <rmcs_dart_guidance/action/filling_command_action.hpp>
#include <rmcs_dart_guidance/action/fire_count_increment_action.hpp>
#include <rmcs_dart_guidance/action/trigger_command_action.hpp>
#include "manager/runtime/manager_types.hpp"
#include "manager/runtime/task.hpp"
#include <rmcs_dart_guidance/resource/mechanism_resources.hpp>

#include <memory>

#include <rmcs_dart_guidance/msg/filling_command.hpp>
#include <rmcs_dart_guidance/msg/trigger_command.hpp>

namespace rmcs_dart_guidance::manager {

class FireAndPreloadTask : public Task {
public:
    FireAndPreloadTask(MechanismResources& resources, ManagerRuntimeState& runtime_state)
        : Task("fire_preload", "发射并预装填") {
        using rmcs_dart_guidance::msg::FillingCommand;
        using rmcs_dart_guidance::msg::TriggerCommand;

        then(std::make_shared<DelayAction>("fire_delay", 50));
        then(
            std::make_shared<TriggerCommandAction>(
                "trigger_free", resources.trigger, TriggerCommand::SERVO_FREE, 200));

        if (runtime_state.fire_count > 0) {
            then(
                std::make_shared<FillingCommandAction>(
                    "filling_lift_up", resources.filling, FillingCommand::LIFT_UP, 200));
            then(
                std::make_shared<FillingCommandAction>(
                    "filling_limit_pulse", resources.filling, FillingCommand::LIMIT_PULSE_FILL,
                    300));
        }

        then(
            std::make_shared<FireCountIncrementAction>(
                "fire_count_increment", runtime_state.fire_count));
    }
};

} // namespace rmcs_dart_guidance::manager
