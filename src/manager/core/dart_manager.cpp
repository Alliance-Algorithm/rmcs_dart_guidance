#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"

#include <chrono>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_dart_guidance::manager {

// Skeleton DartManager after resource cleanup.
// Task/action resources and mechanism IO will be reintroduced in the phase-1 framework.

class DartManager
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartManager()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        register_input("/dart/manager/command", command_input_, false);
        register_output("/dart/manager/fire_count", fire_count_output_, uint32_t{0});
        register_output(
            "/dart/manager/debug/lifecycle_state", debug_lifecycle_state_output_,
            std::string{to_string(ManagerLifecycleState::IDLE)});
        register_output(
            "/dart/manager/debug/current_task", debug_current_task_output_, std::string{});
        register_output(
            "/dart/manager/debug/current_action", debug_current_action_output_, std::string{});
        register_output(
            "/dart/manager/debug/queue", debug_queue_output_, std::vector<ManagerQueuedTaskInfo>{});
        register_output(
            "/dart/manager/debug/last_error", debug_last_error_output_,
            std::optional<ManagerLastErrorInfo>{});

        reset_fire_count();
        sync_debug_outputs();
        RCLCPP_INFO(logger_, "[DartManager] skeleton initialized (resources cleared)");
    }

    void before_updating() override {
        bind_optional_command_inputs();
        sync_debug_outputs();
    }

    void update() override {
        poll_commands();

        if (runtime_state_.lifecycle_state == ManagerLifecycleState::ERROR) {
            sync_debug_outputs();
            return;
        }

        while (runtime_state_.lifecycle_state != ManagerLifecycleState::ERROR) {
            if (!task_state_.current_task) {
                if (task_state_.task_queue.empty()) {
                    if (runtime_state_.lifecycle_state != ManagerLifecycleState::IDLE) {
                        transition_to(ManagerLifecycleState::IDLE);
                    }
                    break;
                }
                dispatch_next_task();
            }

            if (!task_state_.current_task) {
                break;
            }

            const ActionStatus status = tick_current_task();
            if (status == ActionStatus::RUNNING
                || runtime_state_.lifecycle_state == ManagerLifecycleState::ERROR) {
                break;
            }
        }

        sync_debug_outputs();
    }

private:
    struct TaskState {
        std::shared_ptr<Task> current_task;
        std::deque<std::shared_ptr<Task>> task_queue;
        bool first_tick_of_task{true};
    };

    void bind_optional_command_inputs() {
        if (!command_input_.ready()) {
            command_input_.make_and_bind_directly(std::string{});
            RCLCPP_WARN(logger_, "Failed to fetch \"/dart/manager/command\". Set to empty string.");
        }
    }

    void poll_commands() {
        const std::string cmd = command_input_.ready() ? *command_input_ : std::string{};
        if (!cmd.empty()) {
            process_command(cmd);
        }
    }

    void process_command(const std::string& cmd) {
        if (cmd == "cancel") {
            cancel_all();
            return;
        }
        if (cmd == "recover") {
            recover();
            return;
        }

        if (runtime_state_.lifecycle_state == ManagerLifecycleState::ERROR) {
            RCLCPP_WARN(
                logger_, "[DartManager] ignored command '%s' while lifecycle_state=ERROR",
                cmd.c_str());
            return;
        }

        RCLCPP_WARN(
            logger_,
            "[DartManager] command '%s' ignored: task resources not reimplemented yet",
            cmd.c_str());
    }

    void cancel_all() {
        cancel_task_state(task_state_, ActionCancelReason::EXTERNAL_CANCEL);
        transition_to(ManagerLifecycleState::IDLE);
    }

    void recover() {
        if (runtime_state_.lifecycle_state == ManagerLifecycleState::ERROR) {
            reset_task_state(task_state_);
            RCLCPP_INFO(logger_, "[DartManager] recovered from ERROR, state=IDLE");
            transition_to(ManagerLifecycleState::IDLE);
        }
        reset_fire_count();
    }

    void submit_task(std::shared_ptr<Task> task) {
        if (!task) {
            return;
        }
        task_state_.task_queue.push_back(std::move(task));
        RCLCPP_INFO(
            logger_, "[DartManager] task queued: %s (queue size=%zu)",
            task_state_.task_queue.back()->name().c_str(), task_state_.task_queue.size());
    }

    void dispatch_next_task() {
        if (task_state_.task_queue.empty()) {
            return;
        }

        task_state_.current_task = std::move(task_state_.task_queue.front());
        task_state_.task_queue.pop_front();
        task_state_.current_task->bind_runtime_context(
            ActionRuntimeContext{task_state_.current_task->name(), &logger_});

        RCLCPP_INFO(
            logger_, "[DartManager] dispatching task: '%s'",
            task_state_.current_task->name().c_str());
        task_state_.first_tick_of_task = true;
        if (runtime_state_.lifecycle_state != ManagerLifecycleState::RUNNING) {
            transition_to(ManagerLifecycleState::RUNNING);
        }
    }

    ActionStatus tick_current_task() {
        if (!task_state_.current_task) {
            return ActionStatus::SUCCESS;
        }

        const std::string task_name = task_state_.current_task->name();
        const ActionStatus status = task_state_.first_tick_of_task
                                      ? task_state_.current_task->tick_first()
                                      : task_state_.current_task->tick();
        task_state_.first_tick_of_task = false;

        if (status == ActionStatus::SUCCESS) {
            task_state_.current_task->finish_success();
            RCLCPP_INFO(logger_, "[DartManager] task '%s' SUCCESS", task_name.c_str());
            task_state_.current_task.reset();
            if (task_state_.task_queue.empty()) {
                transition_to(ManagerLifecycleState::IDLE);
            }
        } else if (status == ActionStatus::FAILURE) {
            const auto failure = task_state_.current_task->failure_info();
            const std::string failed_action = failure.action_name.empty()
                                                ? task_state_.current_task->name()
                                                : failure.action_name;
            record_last_error(task_state_.current_task->name(), failed_action, failure.reason);
            RCLCPP_ERROR(
                logger_,
                "[DartManager] task '%s' FAILURE at action '%s' reason='%s' -> state=ERROR",
                task_state_.current_task->name().c_str(), failed_action.c_str(),
                to_string(failure.reason));
            on_task_failure();
        }

        return status;
    }

    void on_task_failure() {
        if (!task_state_.current_task) {
            return;
        }
        task_state_.current_task->finish_failure();
        reset_task_state(task_state_);
        transition_to(ManagerLifecycleState::ERROR);
    }

    static void reset_task_state(TaskState& task_state) {
        task_state.current_task.reset();
        task_state.task_queue.clear();
        task_state.first_tick_of_task = true;
    }

    static void cancel_task_state(TaskState& task_state, ActionCancelReason reason) {
        task_state.task_queue.clear();
        if (task_state.current_task) {
            task_state.current_task->cancel(reason);
            task_state.current_task.reset();
        }
        task_state.first_tick_of_task = true;
    }

    void transition_to(ManagerLifecycleState new_state) {
        runtime_state_.lifecycle_state = new_state;
    }

    std::string active_task_name() const {
        return task_state_.current_task ? task_state_.current_task->name() : std::string{};
    }

    std::string active_action_name() const {
        if (!task_state_.current_task) {
            return {};
        }
        return task_state_.current_task->current_action_name();
    }

    void record_last_error(
        const std::string& task_name, const std::string& action_name, ActionFailureReason reason) {
        const auto now = std::chrono::system_clock::now().time_since_epoch();
        auto current_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
        last_error_ = ManagerLastErrorInfo{task_name, action_name, reason, current_time_ms};
    }

    std::vector<ManagerQueuedTaskInfo> build_queue_snapshot() const {
        std::vector<ManagerQueuedTaskInfo> queue;
        queue.reserve(task_state_.task_queue.size());
        for (const auto& task : task_state_.task_queue) {
            if (!task) {
                continue;
            }
            queue.push_back(ManagerQueuedTaskInfo{task->name(), task->description()});
        }
        return queue;
    }

    void sync_debug_outputs() {
        *fire_count_output_ = runtime_state_.fire_count;
        *debug_lifecycle_state_output_ = to_string(runtime_state_.lifecycle_state);
        *debug_current_task_output_ = active_task_name();
        *debug_current_action_output_ = active_action_name();
        *debug_queue_output_ = build_queue_snapshot();
        *debug_last_error_output_ = last_error_;
    }

    void reset_fire_count() { runtime_state_.fire_count = 0; }

    rclcpp::Logger logger_;

    InputInterface<std::string> command_input_;
    OutputInterface<uint32_t> fire_count_output_;
    OutputInterface<std::string> debug_lifecycle_state_output_;
    OutputInterface<std::string> debug_current_task_output_;
    OutputInterface<std::string> debug_current_action_output_;
    OutputInterface<std::vector<ManagerQueuedTaskInfo>> debug_queue_output_;
    OutputInterface<std::optional<ManagerLastErrorInfo>> debug_last_error_output_;

    std::optional<ManagerLastErrorInfo> last_error_;
    ManagerRuntimeState runtime_state_{};
    TaskState task_state_{};
};

} // namespace rmcs_dart_guidance::manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::manager::DartManager, rmcs_executor::Component)
