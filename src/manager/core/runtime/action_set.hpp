#pragma once

#include "manager/core/runtime/action.hpp"

#include <cassert>
#include <memory>
#include <string>
#include <vector>

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// ActionSet
//   并行动作组：将若干 IAction 在同一帧同时推进。
//
//   策略（可在构造时选择）：
//     ALL_SUCCESS —— 所有子动作都 SUCCESS 才返回 SUCCESS；
//                    任意一个 FAILURE 则立刻取消其余并返回 FAILURE。
//     ANY_SUCCESS —— 任意一个 SUCCESS 即返回 SUCCESS，并取消其余；
//                    所有子动作均 FAILURE 才返回 FAILURE。
//
//   ActionSet 本身也是一个 IAction，可以嵌套进 ActionSequence。
// ─────────────────────────────────────────────────────────────────────────────
class ActionSet : public IAction {
public:
    enum class Policy {
        ALL_SUCCESS, // 所有成功才算成功
        ANY_SUCCESS, // 任意成功即算成功
    };

    explicit ActionSet(std::string name, Policy policy = Policy::ALL_SUCCESS)
        : IAction(std::move(name))
        , policy_(policy) {}

    // 追加动作
    ActionSet& also(std::shared_ptr<IAction> action) {
        assert(action && "ActionSet::also: action cannot be null");
        if (action) {
            action->bind_runtime_context(runtime_context());
            entries_.push_back({std::move(action), false, ActionStatus::RUNNING, false});
        }
        return *this;
    }

    void bind_runtime_context(const ActionRuntimeContext& context) override {
        IAction::bind_runtime_context(context);
        for (auto& entry : entries_) {
            if (entry.action) {
                entry.action->bind_runtime_context(context);
            }
        }
    }

    bool should_log_lifecycle() const override { return false; }

    void on_enter() override {
        for (auto& e : entries_) {
            e.done = false;
            e.started = false;
            e.status = ActionStatus::RUNNING;
            if (!e.action) {
                e.done = true; // Skip null actions
            }
        }
    }

    ActionStatus update() override {
        if (entries_.empty()) {
            return policy_ == Policy::ALL_SUCCESS ? ActionStatus::SUCCESS : ActionStatus::FAILURE;
        }

        int running_count = 0;
        int success_count = 0;
        int failure_count = 0;
        ActionFailureInfo first_failure_info;
        bool has_failure = false;

        for (auto& e : entries_) {
            if (e.done || !e.action)
                continue;

            const ActionStatus status = e.started ? e.action->tick() : e.action->tick_first();
            e.started = true;

            e.status = status;

            if (status == ActionStatus::SUCCESS) {
                e.action->finish_success();
                e.done = true;
                e.status = ActionStatus::SUCCESS;
                ++success_count;
                if (policy_ == Policy::ANY_SUCCESS && success_count > 0) {
                    cancel_running(ActionCancelReason::NORMAL_COMPLETION);
                    return ActionStatus::SUCCESS;
                }
            } else if (status == ActionStatus::FAILURE) {
                e.action->finish_failure();
                e.done = true;
                e.status = ActionStatus::FAILURE;
                ++failure_count;
                if (!has_failure) {
                    first_failure_info = e.action->failure_info();
                    has_failure = true;
                }
                if (policy_ == Policy::ALL_SUCCESS && failure_count > 0) {
                    set_failure_info(first_failure_info);
                    cancel_running(ActionCancelReason::DEPENDENCY_FAILURE);
                    return ActionStatus::FAILURE;
                }
            } else {
                ++running_count;
            }
        }

        if (policy_ == Policy::ALL_SUCCESS) {
            return (running_count == 0) ? ActionStatus::SUCCESS : ActionStatus::RUNNING;
        }

        if (running_count == 0) {
            if (has_failure) {
                set_failure_info(first_failure_info);
            } else {
                set_failure_info({name(), ActionFailureReason::DEPENDENCY_FAILURE});
            }
            return ActionStatus::FAILURE;
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override { cancel_running(ActionCancelReason::NORMAL_COMPLETION); }

    void on_failure() override { cancel_running(ActionCancelReason::DEPENDENCY_FAILURE); }

    void on_cancel(ActionCancelReason reason) override { cancel_running(reason); }

private:
    struct Entry {
        std::shared_ptr<IAction> action;
        bool done;
        ActionStatus status;
        bool started;
    };

    void cancel_running(ActionCancelReason reason) {
        for (auto& e : entries_) {
            if (e.started && !e.done && e.action) {
                e.action->cancel(reason);
                e.done = true;
            }
        }
    }

    std::vector<Entry> entries_;
    Policy policy_;
};

} // namespace rmcs_dart_guidance::manager
