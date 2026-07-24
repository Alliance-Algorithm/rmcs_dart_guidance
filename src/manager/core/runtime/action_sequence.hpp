#pragma once

#include "manager/core/runtime/action.hpp"

#include <cassert>
#include <memory>
#include <string>
#include <vector>

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// ActionSequence
//   顺序动作组：将若干 IAction 按加入顺序依次执行。
//
//   - 当前动作返回 SUCCESS 后自动推进到下一个动作
//   - 当前动作返回 FAILURE 时，整组立刻返回 FAILURE（并 cancel 当前动作）
//   - 所有动作均完成后返回 SUCCESS
//
//   ActionSequence 本身也是一个 IAction，因此可以嵌套。
// ─────────────────────────────────────────────────────────────────────────────
class ActionSequence : public IAction {
public:
    explicit ActionSequence(std::string name)
        : IAction(std::move(name)) {}

    ActionSequence& then(std::shared_ptr<IAction> action) {
        assert(action && "ActionSequence::then: action cannot be null");
        if (action) {
            action->bind_runtime_context(runtime_context());
            actions_.push_back(std::move(action));
        }
        return *this;
    }

    void bind_runtime_context(const ActionRuntimeContext& context) override {
        IAction::bind_runtime_context(context);
        for (auto& action : actions_) {
            if (action) {
                action->bind_runtime_context(context);
            }
        }
    }

    bool should_log_lifecycle() const override { return false; }

    void on_enter() override {
        cursor_ = 0;
        first_tick_of_current_ = !actions_.empty();
    }

    ActionStatus update() override {
        if (actions_.empty())
            return ActionStatus::SUCCESS;

        while (cursor_ < actions_.size()) {
            auto& action_ptr = actions_[cursor_];
            if (!action_ptr) {
                ++cursor_;
                first_tick_of_current_ = true;
                continue;
            }

            auto& current = *action_ptr;
            const ActionStatus status =
                first_tick_of_current_ ? current.tick_first() : current.tick();
            first_tick_of_current_ = false;

            if (status == ActionStatus::RUNNING) {
                return ActionStatus::RUNNING;
            }

            if (status == ActionStatus::SUCCESS) {
                current.finish_success();
                ++cursor_;
                first_tick_of_current_ = true;
                continue;
            }

            set_failure_info(current.failure_info());
            current.finish_failure();
            return ActionStatus::FAILURE;
        }

        return ActionStatus::SUCCESS;
    }

    void on_exit() override { cancel_active_child(ActionCancelReason::NORMAL_COMPLETION); }

    void on_failure() override { cancel_active_child(ActionCancelReason::DEPENDENCY_FAILURE); }

    void on_cancel(ActionCancelReason reason) override { cancel_active_child(reason); }

    std::size_t size() const { return actions_.size(); }

    std::size_t cursor() const { return cursor_; }

    std::string current_action_name() const {
        if (cursor_ >= actions_.size()) {
            return {};
        }

        const auto& action_ptr = actions_[cursor_];
        return action_ptr ? action_ptr->name() : std::string{};
    }

private:
    void cancel_active_child(ActionCancelReason reason) {
        if (cursor_ < actions_.size()) {
            auto& action_ptr = actions_[cursor_];
            if (action_ptr && action_ptr->is_active()) {
                action_ptr->cancel(reason);
            }
        }
    }
    std::vector<std::shared_ptr<IAction>> actions_;
    std::size_t cursor_{0};
    bool first_tick_of_current_{true};
};

} // namespace rmcs_dart_guidance::manager
