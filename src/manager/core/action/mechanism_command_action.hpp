#pragma once

#include "manager/core/runtime/action.hpp"
#include "manager/resources/belt_resource.hpp"
#include "manager/resources/filling_resource.hpp"
#include "manager/resources/trigger_resource.hpp"

#include <cstdint>
#include <limits>
#include <string>
#include <utility>

#include <rmcs_dart_guidance/msg/belt_command.hpp>
#include <rmcs_dart_guidance/msg/filling_command.hpp>
#include <rmcs_dart_guidance/msg/trigger_command.hpp>

namespace rmcs_dart_guidance::manager {

template <typename Resource, typename Command>
class MechanismCommandAction : public IAction {
public:
    // timeout_ticks: 0 disables timeout (debug only). Business tasks must pass a positive value.
    MechanismCommandAction(
        std::string name, Resource& resource, Command command, uint64_t timeout_ticks)
        : IAction(std::move(name))
        , resource_(resource)
        , command_(command)
        , timeout_ticks_(timeout_ticks) {}

    void on_enter() override { request_command(); }

    ActionStatus update() override {
        if (resource_.failed()) {
            return fail(ActionFailureReason::DEPENDENCY_FAILURE);
        }
        if (resource_.succeeded()) {
            return ActionStatus::SUCCESS;
        }
        if (timeout_ticks_ > 0 && elapsed_ticks() >= timeout_ticks_) {
            return fail(ActionFailureReason::TIMEOUT);
        }
        return ActionStatus::RUNNING;
    }

    // 成功：写 IDLE。失败/外部取消：只写 ABORT，保持到 Manager recover 再 idle。
    void on_exit() override { resource_.idle(); }

    void on_failure() override { resource_.abort(); }

    void on_cancel(ActionCancelReason reason) override {
        if (reason == ActionCancelReason::NORMAL_COMPLETION) {
            resource_.idle();
            return;
        }
        resource_.abort();
    }

protected:
    virtual void request_command() { resource_.request(command_); }

    Resource& resource_;
    Command command_;
    uint64_t timeout_ticks_;
};

class BeltCommandAction
    : public MechanismCommandAction<BeltResource, rmcs_dart_guidance::msg::BeltCommand> {
public:
    using MechanismCommandAction::MechanismCommandAction;
};

class FillingCommandAction
    : public MechanismCommandAction<FillingResource, rmcs_dart_guidance::msg::FillingCommand> {
public:
    using MechanismCommandAction::MechanismCommandAction;
};

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
