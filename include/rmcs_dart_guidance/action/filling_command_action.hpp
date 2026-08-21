#pragma once

#include <rmcs_dart_guidance/action/mechanism_command_action.hpp>
#include <rmcs_dart_guidance/resource/filling_resource.hpp>

#include <rmcs_dart_guidance/msg/filling_command.hpp>

namespace rmcs_dart_guidance::manager {

class FillingCommandAction
    : public MechanismCommandAction<FillingResource, rmcs_dart_guidance::msg::FillingCommand> {
public:
    using MechanismCommandAction::MechanismCommandAction;
};

} // namespace rmcs_dart_guidance::manager
