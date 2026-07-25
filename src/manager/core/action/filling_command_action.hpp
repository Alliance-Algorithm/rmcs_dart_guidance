#pragma once

#include "manager/core/action/mechanism_command_action.hpp"
#include "manager/resources/filling_resource.hpp"

#include <rmcs_dart_guidance/msg/filling_command.hpp>

namespace rmcs_dart_guidance::manager {

class FillingCommandAction
    : public MechanismCommandAction<FillingResource, rmcs_dart_guidance::msg::FillingCommand> {
public:
    using MechanismCommandAction::MechanismCommandAction;
};

} // namespace rmcs_dart_guidance::manager
