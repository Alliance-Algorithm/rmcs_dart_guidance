#pragma once

#include "manager/runtime/action_set.hpp"
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

class DartInitTask : public Task {
public:
    explicit DartInitTask(MechanismResources& resources)
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
        // init->also(
        //     std::make_shared<TriggerCommandAction>(
        //         "trigger_free", resources.trigger, TriggerCommand::TRIGGER_FREE,
        //         kServoTimeoutTicks));
        then(init);
    }

private:
    static constexpr uint64_t kMechanismTimeoutTicks = 20000;
    static constexpr uint64_t kServoTimeoutTicks = 1000;
};

} // namespace rmcs_dart_guidance::manager
