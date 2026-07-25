#pragma once

#include <rmcs_dart_guidance/action/mechanism_command_action.hpp>
#include <rmcs_dart_guidance/resource/example_resource.hpp>

#include <rmcs_dart_guidance/msg/example_command.hpp>

namespace rmcs_dart_guidance::manager {

class ExampleCommandAction
    : public MechanismCommandAction<ExampleResource, rmcs_dart_guidance::msg::ExampleCommand> {
public:
    using MechanismCommandAction::MechanismCommandAction;
};

} // namespace rmcs_dart_guidance::manager
