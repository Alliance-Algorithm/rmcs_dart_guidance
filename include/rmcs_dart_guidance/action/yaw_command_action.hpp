#pragma once

#include <rmcs_dart_guidance/action/mechanism_command_action.hpp>
#include <rmcs_dart_guidance/resource/yaw_resource.hpp>

#include <limits>
#include <string>
#include <utility>

#include <rmcs_dart_guidance/msg/yaw_command.hpp>

namespace rmcs_dart_guidance::manager {

class YawCommandAction
    : public MechanismCommandAction<YawResource, rmcs_dart_guidance::msg::YawCommand> {
public:
    YawCommandAction(
        std::string name, YawResource& resource, rmcs_dart_guidance::msg::YawCommand command,
        uint64_t timeout_ticks,
        double target_setpoint = std::numeric_limits<double>::quiet_NaN())
        : MechanismCommandAction(std::move(name), resource, command, timeout_ticks)
        , target_setpoint_(target_setpoint) {}

protected:
    void request_command() override { resource_.request(command_, target_setpoint_); }

private:
    double target_setpoint_;
};

} // namespace rmcs_dart_guidance::manager
