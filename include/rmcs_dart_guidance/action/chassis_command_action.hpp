#pragma once

#include <rmcs_dart_guidance/action/mechanism_command_action.hpp>
#include <rmcs_dart_guidance/resource/chassis_resource.hpp>

#include <rmcs_dart_guidance/msg/chassis_command.hpp>

namespace rmcs_dart_guidance::manager {

class ChassisCommandAction
    : public MechanismCommandAction<ChassisResource, rmcs_dart_guidance::msg::ChassisCommand> {
public:
    using MechanismCommandAction::MechanismCommandAction;
};

} // namespace rmcs_dart_guidance::manager
