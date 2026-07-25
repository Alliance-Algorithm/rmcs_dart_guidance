#pragma once

#include "manager/core/runtime/action.hpp"

#include <cstdint>
#include <string>
#include <utility>

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

} // namespace rmcs_dart_guidance::manager
