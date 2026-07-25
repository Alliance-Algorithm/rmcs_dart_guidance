#pragma once

#include "manager/core/action/mechanism_command_action.hpp"
#include "manager/resources/trigger_resource.hpp"

#include <limits>
#include <string>
#include <utility>

#include <rmcs_dart_guidance/msg/trigger_command.hpp>

namespace rmcs_dart_guidance::manager {

class TriggerCommandAction
    : public MechanismCommandAction<TriggerResource, rmcs_dart_guidance::msg::TriggerCommand> {
public:
    TriggerCommandAction(
        std::string name, TriggerResource& resource,
        rmcs_dart_guidance::msg::TriggerCommand command, uint64_t timeout_ticks,
        double setpoint = std::numeric_limits<double>::quiet_NaN())
        : MechanismCommandAction(std::move(name), resource, command, timeout_ticks)
        , setpoint_(setpoint) {}

protected:
    void request_command() override { resource_.request(command_, setpoint_); }

private:
    double setpoint_;
};

} // namespace rmcs_dart_guidance::manager
