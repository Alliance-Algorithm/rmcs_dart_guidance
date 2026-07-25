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

} // namespace rmcs_dart_guidance::manager
