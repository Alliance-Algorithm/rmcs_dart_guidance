#pragma once

#include "manager/runtime/action.hpp"
#include <rmcs_dart_guidance/resource/yaw_resource.hpp>

#include <limits>
#include <string>
#include <utility>

#include <rmcs_dart_guidance/msg/yaw_command.hpp>

namespace rmcs_dart_guidance::manager {

class YawIssueCommandAction : public IAction {
public:
    YawIssueCommandAction(
        std::string name, YawResource& resource, rmcs_dart_guidance::msg::YawCommand command,
        double setpoint = std::numeric_limits<double>::quiet_NaN())
        : IAction(std::move(name))
        , resource_(resource)
        , command_(command)
        , setpoint_(setpoint) {}

    bool should_log_lifecycle() const override { return false; }

    void on_enter() override { resource_.request(command_, setpoint_); }

    ActionStatus update() override { return ActionStatus::SUCCESS; }

private:
    YawResource& resource_;
    rmcs_dart_guidance::msg::YawCommand command_;
    double setpoint_;
};

} // namespace rmcs_dart_guidance::manager
