#pragma once

#include "manager/core/action/mechanism_command_action.hpp"
#include "manager/resources/belt_resource.hpp"

#include <rmcs_dart_guidance/msg/belt_command.hpp>

namespace rmcs_dart_guidance::manager {

class BeltCommandAction
    : public MechanismCommandAction<BeltResource, rmcs_dart_guidance::msg::BeltCommand> {
public:
    using MechanismCommandAction::MechanismCommandAction;
};

} // namespace rmcs_dart_guidance::manager
