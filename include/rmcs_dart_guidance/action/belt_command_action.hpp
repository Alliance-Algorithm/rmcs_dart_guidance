#pragma once

#include <rmcs_dart_guidance/action/mechanism_command_action.hpp>
#include <rmcs_dart_guidance/resource/belt_resource.hpp>

#include <rmcs_dart_guidance/msg/belt_command.hpp>

namespace rmcs_dart_guidance::manager {

class BeltCommandAction
    : public MechanismCommandAction<BeltResource, rmcs_dart_guidance::msg::BeltCommand> {
public:
    using MechanismCommandAction::MechanismCommandAction;
};

class TimedBeltCommandAction : public BeltCommandAction {
public:
    using BeltCommandAction::BeltCommandAction;

    ActionStatus update() override {
        if (resource_.failed()) {
            return fail(ActionFailureReason::DEPENDENCY_FAILURE);
        }
        if (timeout_ticks_ == 0 || elapsed_ticks() >= timeout_ticks_) {
            return ActionStatus::SUCCESS;
        }
        return ActionStatus::RUNNING;
    }
};

} // namespace rmcs_dart_guidance::manager
