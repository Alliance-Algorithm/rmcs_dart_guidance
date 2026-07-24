#pragma once

#include <cstdint>
#include <string>
#include <utility>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// ActionStatus
//   动作的生命周期状态，由 Action::update() 每帧返回。
// ─────────────────────────────────────────────────────────────────────────────
enum class ActionStatus { RUNNING, SUCCESS, FAILURE };

enum class ActionFailureReason : uint8_t {
    NONE,
    TIMEOUT,
    STALL,
    INVALID_INPUT,
    STALE_INPUT,
    CONFIGURATION_ERROR,
    EXTERNAL_CANCEL,
    DEPENDENCY_FAILURE,
};

enum class ActionCancelReason : uint8_t {
    EXTERNAL_CANCEL,
    NORMAL_COMPLETION,
    DEPENDENCY_FAILURE,
};

inline const char* to_string(ActionFailureReason reason) {
    switch (reason) {
    case ActionFailureReason::NONE: return "none";
    case ActionFailureReason::TIMEOUT: return "timeout";
    case ActionFailureReason::STALL: return "stall";
    case ActionFailureReason::INVALID_INPUT: return "invalid_input";
    case ActionFailureReason::STALE_INPUT: return "stale_input";
    case ActionFailureReason::CONFIGURATION_ERROR: return "configuration_error";
    case ActionFailureReason::EXTERNAL_CANCEL: return "external_cancel";
    case ActionFailureReason::DEPENDENCY_FAILURE: return "dependency_failure";
    }
    return "unknown";
}

inline const char* to_string(ActionCancelReason reason) {
    switch (reason) {
    case ActionCancelReason::EXTERNAL_CANCEL: return "external_cancel";
    case ActionCancelReason::NORMAL_COMPLETION: return "normal_completion";
    case ActionCancelReason::DEPENDENCY_FAILURE: return "dependency_failure";
    }
    return "unknown";
}

struct ActionFailureInfo {
    std::string action_name;
    ActionFailureReason reason{ActionFailureReason::NONE};
};

struct ActionRuntimeContext {
    std::string task_name;
    const rclcpp::Logger* logger{nullptr};
};

class IAction {
public:
    explicit IAction(std::string name)
        : name_(std::move(name)) {}

    virtual ~IAction() = default;

    // 禁止拷贝和移动，Action 通过 shared_ptr 管理
    IAction(const IAction&) = delete;
    IAction& operator=(const IAction&) = delete;
    IAction(IAction&&) = delete;
    IAction& operator=(IAction&&) = delete;

    // ── 供子类重写 ────────────────────────────────────────────────────────────

    // 进入时调用，可在此初始化内部计数器、记录起始时刻等
    virtual void on_enter() {}

    // 每帧调用，返回执行状态
    virtual ActionStatus update() = 0;

    // 正常成功退出时调用，可在此做清理（如写 IDLE）
    virtual void on_exit() {}

    // 失败退出时调用；默认与 on_exit 一致，机构动作可覆写为 ABORT 保持
    virtual void on_failure() { on_exit(); }

    // 被外部取消时调用，默认与 on_exit 一致，容器动作可覆写以传播取消原因
    virtual void on_cancel(ActionCancelReason reason) {
        (void)reason;
        on_exit();
    }

    virtual void bind_runtime_context(const ActionRuntimeContext& context) {
        runtime_context_ = context;
    }

    virtual bool should_log_lifecycle() const { return true; }

    // ── 供外部调用 ────────────────────────────────────────────────────────────

    const std::string& name() const { return name_; }

    // 获取动作已运行的帧数
    uint64_t elapsed_ticks() const { return elapsed_ticks_; }

    const ActionRuntimeContext& runtime_context() const { return runtime_context_; }

    const ActionFailureInfo& failure_info() const { return failure_info_; }

    bool is_active() const { return started_ && !finished_; }

    // ── 框架内部使用（由 ActionSequence / ActionSet 调用）───────────────────

    // 初始化并执行第一帧，返回状态
    ActionStatus tick_first() {
        elapsed_ticks_ = 0;
        started_ = true;
        finished_ = false;
        clear_failure_info();
        on_enter();
        log_start();
        return tick();
    }

    // 执行后续帧
    ActionStatus tick() {
        ++elapsed_ticks_;
        const ActionStatus status = update();

        if (status == ActionStatus::SUCCESS) {
            log_success();
        } else if (status == ActionStatus::FAILURE) {
            if (failure_info_.action_name.empty()) {
                failure_info_.action_name = name_;
            }
            log_failure();
        }

        return status;
    }

    void finish_success() {
        if (!is_active())
            return;

        finished_ = true;
        on_exit();
        clear_failure_info();
    }

    void finish_failure() {
        if (!is_active())
            return;

        finished_ = true;
        on_failure();
    }

    // 强制退出（被取消时使用）
    void cancel(ActionCancelReason reason = ActionCancelReason::EXTERNAL_CANCEL) {
        if (!is_active())
            return;

        log_cancel(reason);
        finished_ = true;
        on_cancel(reason);
    }

protected:
    ActionStatus fail(ActionFailureReason reason) {
        set_failure_info({name_, reason});
        return ActionStatus::FAILURE;
    }

    void set_failure_info(ActionFailureInfo info) {
        if (info.action_name.empty()) {
            info.action_name = name_;
        }
        failure_info_ = std::move(info);
    }

    void clear_failure_info() { failure_info_ = ActionFailureInfo{}; }

private:
    void log_start() const {
        if (!should_log_lifecycle() || runtime_context_.logger == nullptr)
            return;

        RCLCPP_INFO(
            *runtime_context_.logger, "[ActionRuntime] task='%s' action='%s' start",
            runtime_context_.task_name.c_str(), name_.c_str());
    }

    void log_success() const {
        if (!should_log_lifecycle() || runtime_context_.logger == nullptr)
            return;

        RCLCPP_INFO(
            *runtime_context_.logger, "[ActionRuntime] task='%s' action='%s' success",
            runtime_context_.task_name.c_str(), name_.c_str());
    }

    void log_failure() const {
        if (!should_log_lifecycle() || runtime_context_.logger == nullptr)
            return;

        RCLCPP_ERROR(
            *runtime_context_.logger, "[ActionRuntime] task='%s' action='%s' failure reason='%s'",
            runtime_context_.task_name.c_str(), failure_info_.action_name.c_str(),
            to_string(failure_info_.reason));
    }

    void log_cancel(ActionCancelReason reason) const {
        if (!should_log_lifecycle() || runtime_context_.logger == nullptr)
            return;

        RCLCPP_WARN(
            *runtime_context_.logger, "[ActionRuntime] task='%s' action='%s' cancel reason='%s'",
            runtime_context_.task_name.c_str(), name_.c_str(), to_string(reason));
    }

    std::string name_;
    uint64_t elapsed_ticks_{0};
    ActionRuntimeContext runtime_context_;
    ActionFailureInfo failure_info_;
    bool started_{false};
    bool finished_{true};
};

} // namespace rmcs_dart_guidance::manager
