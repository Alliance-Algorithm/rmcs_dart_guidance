#pragma once

#include "manager/core/runtime/task.hpp"

#include <functional>
#include <memory>
#include <string>
#include <utility>

namespace rmcs_dart_guidance::manager {

class DeferredTaskAction : public IAction {
public:
    using TaskFactory = std::function<std::shared_ptr<Task>()>;

    DeferredTaskAction(std::string name, TaskFactory task_factory)
        : IAction(std::move(name))
        , task_factory_(std::move(task_factory)) {}

    void on_enter() override {
        task_started_ = false;
        task_.reset();

        if (task_factory_) {
            task_ = task_factory_();
        }

        if (task_) {
            task_->bind_runtime_context(runtime_context());
        }
    }

    ActionStatus update() override {
        if (!task_) {
            return fail(ActionFailureReason::CONFIGURATION_ERROR);
        }

        const ActionStatus status = task_started_ ? task_->tick() : task_->tick_first();
        task_started_ = true;

        if (status == ActionStatus::SUCCESS) {
            task_->finish_success();
        } else if (status == ActionStatus::FAILURE) {
            set_failure_info(task_->failure_info());
            task_->finish_failure();
        }

        return status;
    }

    void on_exit() override { cancel_child(ActionCancelReason::NORMAL_COMPLETION); }

    void on_cancel(ActionCancelReason reason) override { cancel_child(reason); }

    bool should_log_lifecycle() const override { return false; }

private:
    void cancel_child(ActionCancelReason reason) {
        if (task_ && task_->is_active()) {
            task_->cancel(reason);
        }
    }

    TaskFactory task_factory_;
    std::shared_ptr<Task> task_;
    bool task_started_{false};
};

} // namespace rmcs_dart_guidance::manager
