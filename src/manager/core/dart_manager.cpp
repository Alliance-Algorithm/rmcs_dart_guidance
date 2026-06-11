#include "manager/core/runtime/task.hpp"
#include "manager/resources/task_factory.hpp"
#include "rmcs_msgs/dart_motor_exit_mode.hpp"
#include "rmcs_msgs/dart_servo_command.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int64.hpp>

namespace rmcs_dart_guidance::manager {

class DartManager
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartManager()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        // belt
        register_output("/dart_manager/belt/command", belt_command_);
        register_output("/dart_manager/belt/target_velocity", belt_target_velocity_);
        register_output("/dart_manager/belt/exit_mode", belt_exit_mode_);
        register_output(
            "/dart_manager/belt/max_torque_override", belt_max_torque_override_,
            std::numeric_limits<double>::quiet_NaN());
        register_input("/dart/drive_belt/left/angle", belt_left_angle_);
        register_input("/dart/drive_belt/left/velocity", belt_left_velocity_);
        register_input("/dart/drive_belt/left/torque", belt_left_torque_);
        register_input("/dart/drive_belt/right/angle", belt_right_angle_);
        register_input("/dart/drive_belt/right/velocity", belt_right_velocity_);
        register_input("/dart/drive_belt/right/torque", belt_right_torque_);

        belt_down_velocity_ = get_parameter("belt_down_velocity").as_double();
        belt_down_travel_angle_ = get_parameter("belt_down_travel_angle").as_double();
        belt_up_velocity_ = get_parameter("belt_up_velocity").as_double();
        belt_up_travel_angle_ = get_parameter("belt_up_travel_angle").as_double();
        belt_interference_relief_travel_angle_ =
            get_parameter("belt_interference_relief_travel_angle").as_double();
        belt_init_velocity_ = get_parameter("belt_init_velocity").as_double();
        manual_belt_velocity_ = get_parameter("manual_max_velocity").as_double();
        belt_stall_velocity_threshold_ = get_parameter("belt_stall_velocity_threshold").as_double();
        belt_stall_torque_threshold_ = get_parameter("belt_stall_torque_threshold").as_double();
        belt_stall_confirm_ticks_ =
            static_cast<uint64_t>(get_parameter("belt_stall_confirm_ticks").as_int());
        belt_init_stall_velocity_threshold_ =
            get_parameter("belt_init_stall_velocity_threshold").as_double();
        belt_init_stall_torque_threshold_ =
            get_parameter("belt_init_stall_torque_threshold").as_double();
        belt_init_stall_confirm_ticks_ =
            static_cast<uint64_t>(get_parameter("belt_init_stall_confirm_ticks").as_int());
        belt_init_max_torque_ = get_parameter("belt_init_max_torque").as_double();

        // lift
        register_output("/dart_manager/lift/command", lift_command_);
        register_output("/dart_manager/lift/target_velocity", lift_target_velocity_);
        register_output("/dart_manager/lift/exit_mode", lift_exit_mode_);
        register_input("/dart/lifting_left/velocity", lift_left_velocity_);
        register_input("/dart/lifting_left/torque", lift_left_torque_);
        register_input("/dart/lifting_right/velocity", lift_right_velocity_);
        register_input("/dart/lifting_right/torque", lift_right_torque_);

        lift_velocity_ = get_parameter("lift_velocity").as_double();
        lift_stall_velocity_threshold_ =
            get_parameter("lifting_velocity_stall_threshold").as_double();
        lift_stall_torque_threshold_ = get_parameter("lifting_stall_torque_threshold").as_double();
        lift_stall_confirm_ticks_ =
            static_cast<uint64_t>(get_parameter("lifting_stall_confirm_ticks").as_int());
        lift_stall_min_run_ticks_ =
            static_cast<uint64_t>(get_parameter("lifting_stall_min_run_ticks").as_int());

        // carriage
        register_output("/dart_manager/carriage/command", carriage_command_);
        register_output("/dart_manager/carriage/target_velocity", carriage_target_velocity_);
        register_output(
            "/dart_manager/carriage/target_angle", carriage_target_angle_,
            std::numeric_limits<double>::quiet_NaN());
        register_output("/dart_manager/carriage/origin_angle", carriage_origin_angle_);
        register_input("/dart/force_screw_motor/angle", force_screw_angle_);
        register_input("/dart/force_screw_motor/encoder_angle", force_screw_encoder_angle_);
        register_input("/dart/force_screw_motor/velocity", force_screw_velocity_);
        register_input("/dart/force_screw_motor/torque", force_screw_torque_);

        fire_target_ = get_parameter("fire_target").as_string();
        basement_travel_angle_ = get_parameter("basement_travel_angle").as_double();
        frontier_travel_angle_ = get_parameter("frontier_travel_angle").as_double();
        carriage_down_velocity_ = get_parameter("carriage_down_velocity").as_double();
        carriage_up_velocity_ = get_parameter("carriage_up_velocity").as_double();
        carriage_lift_down_limit_ = get_parameter("carriage_lift_down_limit").as_double();
        carriage_startup_position_angle_ =
            get_parameter("carriage_startup_position_angle").as_double();
        carriage_adjust_down_angle_ = get_parameter("carriage_adjust_down_angle").as_double();
        carriage_adjust_up_angle_ = get_parameter("carriage_adjust_up_angle").as_double();
        carriage_stall_velocity_threshold_ =
            get_parameter("carriage_stall_velocity_threshold").as_double();
        carriage_stall_torque_threshold_ =
            get_parameter("carriage_stall_torque_threshold").as_double();
        carriage_stall_confirm_ticks_ =
            static_cast<uint64_t>(get_parameter("carriage_stall_confirm_ticks").as_int());
        carriage_calibration_velocity_ = get_parameter("carriage_calibration_velocity").as_double();
        carriage_calibration_stall_velocity_threshold_ =
            get_parameter("carriage_calibration_stall_velocity_threshold").as_double();
        carriage_calibration_stall_torque_threshold_ =
            get_parameter("carriage_calibration_stall_torque_threshold").as_double();
        carriage_calibration_stall_confirm_ticks_ = static_cast<uint64_t>(
            get_parameter("carriage_calibration_stall_confirm_ticks").as_int());
        carriage_calibration_max_torque_ =
            get_parameter("carriage_calibration_max_torque").as_double();
        carriage_calibration_parking_angle_ =
            get_parameter("carriage_calibration_parking_angle").as_double();
        carriage_angle_allowable_error_ =
            get_parameter("carriage_angle_allowable_error").as_double();
        carriage_min_run_ticks_ =
            static_cast<uint64_t>(get_parameter("carriage_min_run_ticks").as_int());
        carriage_timeout_ticks_ =
            static_cast<uint64_t>(get_parameter("carriage_timeout_ticks").as_int());
        // trigger
        register_output("/dart_manager/trigger/command", trigger_command_);

        // limit servo
        register_output("/dart_manager/limit_servo/command", limiting_command_);
        limiting_fill_ticks_ = (uint64_t)get_parameter("limiting_fill_ticks").as_int();

        // yaw pitch force
        register_output("/dart_manager/force/error", force_error_, int32_t{0});
        register_output(
            "/dart_manager/force/max_velocity_override", force_max_velocity_override_,
            std::numeric_limits<double>::quiet_NaN());
        register_output(
            "/dart_manager/force/max_torque_override", force_max_torque_override_,
            std::numeric_limits<double>::quiet_NaN());
        register_output(
            "/dart_manager/angle/error_vector", angle_error_vector_, Eigen::Vector2d::Zero());
        register_output(
            "/dart_manager/chassis_leveling/pitch/flag", chassis_pitch_leveling_flag_, false);
        register_output(
            "/dart_manager/chassis_leveling/roll/flag", chassis_roll_leveling_flag_, false);

        register_input("/force_sensor/channel_1/weight", force_sensor_ch1_);
        register_input("/force_sensor/channel_2/weight", force_sensor_ch2_);

        // chassis leveling
        register_input("/dart/leveling_feet/front_left/velocity", leveling_front_left_velocity_);
        register_input("/dart/leveling_feet/front_left/torque", leveling_front_left_torque_);
        register_input("/dart/leveling_feet/front_right/velocity", leveling_front_right_velocity_);
        register_input("/dart/leveling_feet/front_right/torque", leveling_front_right_torque_);
        register_input("/dart/leveling_feet/rear_left/velocity", leveling_rear_left_velocity_);
        register_input("/dart/leveling_feet/rear_left/torque", leveling_rear_left_torque_);
        register_input("/dart/leveling_feet/rear_right/velocity", leveling_rear_right_velocity_);
        register_input("/dart/leveling_feet/rear_right/torque", leveling_rear_right_torque_);

        // vision
        register_input("/dart_guidance/camera/target_position", current_target_input_, false);
        register_input("/dart_guidance/tracker/tracking", tracking_input_, false);
        register_input("/dart_guidance/camera/target_seq", target_seq_input_, false);
        register_input("/imu/catapult_pitch_angle", pitch_angle_);
        register_input("/imu/catapult_roll_angle", roll_angle_);

        // manual control
        register_input("/remote/switch/left", remote_left_switch_, false);
        register_input("/remote/switch/right", remote_right_switch_, false);
        register_input("/remote/rotary_knob_switch", remote_rotary_knob_switch_, false);
        register_input("/remote/joystick/left", remote_left_joystick_, false);
        register_input("/remote/joystick/right", remote_right_joystick_, false);

        manual_angle_max_error_ = get_parameter("angle_manual_scale").as_double();
        manual_force_max_error_ =
            static_cast<int32_t>(std::lround(get_parameter("force_manual_scale").as_double()));

        vision_aim_profile_provider_.load_from(*this);
        if (!vision_aim_profile_provider_.valid()) {
            RCLCPP_WARN(
                logger_,
                "[DartManager] vision aim profiles unavailable: %s. "
                "launch_prepare_with_vision will fail with configuration/input errors until fixed.",
                vision_aim_profile_provider_.error_message().c_str());
        }

        // command/debug io
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
            "/dart/manager/debug/manual_control_active", debug_manual_control_active_output_,
            false);
        register_output(
            "/dart/manager/debug/queue", debug_queue_output_, std::vector<ManagerQueuedTaskInfo>{});
        register_output(
            "/dart/manager/debug/last_error", debug_last_error_output_,
            std::optional<ManagerLastErrorInfo>{});

        reset_fire_count();
        sync_debug_outputs();
        RCLCPP_INFO(logger_, "[DartManager] initialized");
    }

    void before_updating() override {
        bind_optional_command_inputs();
        bind_optional_manual_inputs();
        bind_optional_vision_inputs();

        auto input = input_context();
        auto output = output_context();
        auto manager_settings = settings();
        submit_task(make_dart_init_task(input, output, manager_settings));

        if (fire_target_ == "basement") {
            carriage_travel_angle_ = basement_travel_angle_;
        } else if (fire_target_ == "frontier") {
            carriage_travel_angle_ = frontier_travel_angle_;
        } else {
            RCLCPP_WARN(logger_, "Invalid fire_target '%s'", fire_target_.c_str());
            carriage_travel_angle_ = 0.0;
        }
        *carriage_origin_angle_ = 0.0;

        sync_debug_outputs();
        RCLCPP_INFO(logger_, "[DartManager] queued BeltInitTask on startup");
    }

    void update() override {
        initialize_carriage_origin_from_startup_position();
        if (!carriage_startup_position_initialized_) {
            sync_debug_outputs();
            return;
        }
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

        if (log_counter_ % 1000 == 0) {
            log_counter_ = 0;
            RCLCPP_INFO(
                get_logger(), "[carriage]:origin:%f | current: %f | delta: %f",
                *carriage_origin_angle_, *force_screw_encoder_angle_,
                *force_screw_encoder_angle_ - *carriage_origin_angle_);
        }
        log_counter_++;
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

    void bind_optional_manual_inputs() {
        if (!remote_left_switch_.ready()) {
            remote_left_switch_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(logger_, "Failed to fetch \"/remote/switch/left\". Set to UNKNOWN.");
        }

        if (!remote_right_switch_.ready()) {
            remote_right_switch_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(logger_, "Failed to fetch \"/remote/switch/right\". Set to UNKNOWN.");
        }

        if (!remote_rotary_knob_switch_.ready()) {
            remote_rotary_knob_switch_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(logger_, "Failed to fetch \"/remote/rotary_knob_switch\". Set to UNKNOWN.");
        }

        if (!remote_left_joystick_.ready()) {
            remote_left_joystick_.make_and_bind_directly(Eigen::Vector2d::Zero());
            RCLCPP_WARN(logger_, "Failed to fetch \"/remote/joystick/left\". Set to zero.");
        }

        if (!remote_right_joystick_.ready()) {
            remote_right_joystick_.make_and_bind_directly(Eigen::Vector2d::Zero());
            RCLCPP_WARN(logger_, "Failed to fetch \"/remote/joystick/right\". Set to zero.");
        }
    }

    void bind_optional_vision_inputs() {
        if (!current_target_input_.ready()) {
            current_target_input_.make_and_bind_directly(cv::Point2i(-1, -1));
            RCLCPP_WARN(
                logger_,
                "Failed to fetch \"/dart_guidance/camera/target_position\". Set to (-1, -1).");
        }

        if (!tracking_input_.ready()) {
            tracking_input_.make_and_bind_directly(false);
            RCLCPP_WARN(
                logger_, "Failed to fetch \"/dart_guidance/tracker/tracking\". Set to false.");
        }

        if (!target_seq_input_.ready()) {
            target_seq_input_.make_and_bind_directly(uint64_t{0});
            RCLCPP_WARN(
                logger_, "Failed to fetch \"/dart_guidance/camera/target_seq\". Set to zero.");
        }
    }

    void poll_commands() {
        const std::string cmd = command_input_.ready() ? *command_input_ : std::string{};
        if (!cmd.empty()) {
            process_command(cmd);
        }
    }

    void initialize_carriage_origin_from_startup_position() {
        if (carriage_startup_position_initialized_) {
            return;
        }
        if (!force_screw_encoder_angle_.ready()) {
            return;
        }

        const double current_encoder_angle = *force_screw_encoder_angle_;
        const double computed_origin_angle =
            current_encoder_angle - carriage_startup_position_angle_;

        *carriage_origin_angle_ = computed_origin_angle;
        runtime_state_.carriage_power_cycle_origin_angle = computed_origin_angle;
        carriage_startup_position_initialized_ = true;

        RCLCPP_INFO(
            logger_,
            "[DartManager] initialized carriage origin from startup position:"
            " encoder=%.6f startup_position=%.6f origin=%.6f",
            current_encoder_angle, carriage_startup_position_angle_, computed_origin_angle);
    }

    void process_command(const std::string& cmd) {
        if (cmd == "cancel") {
            cancel_all();
            return;
        } else if (cmd == "recover") {
            recover();
            return;
        }

        if (runtime_state_.lifecycle_state == ManagerLifecycleState::ERROR) {
            RCLCPP_WARN(
                logger_, "[DartManager] ignored command '%s' while lifecycle_state=ERROR",
                cmd.c_str());
            return;
        }

        auto input = input_context();
        auto output = output_context();
        auto manager_settings = settings();
        auto task = make_task(
            cmd, input, output, manager_settings, vision_aim_profile_provider_, runtime_state_);

        RCLCPP_INFO(logger_, "[DartManager] received command: '%s'", cmd.c_str());
        if (task) {
            submit_task(std::move(task));
        } else {
            RCLCPP_WARN(logger_, "[DartManager] unknown command: '%s'", cmd.c_str());
        }
    }

    void cancel_all() {
        cancel_task_state(task_state_, ActionCancelReason::EXTERNAL_CANCEL);
        reset_control_outputs();

        // RCLCPP_WARN(logger_, "[DartManager] all tasks cancelled");
        transition_to(ManagerLifecycleState::IDLE);
    }

    void recover() {
        if (runtime_state_.lifecycle_state == ManagerLifecycleState::ERROR) {
            reset_task_state(task_state_);
            RCLCPP_INFO(logger_, "[DartManager] recovered from ERROR, state=IDLE");
            transition_to(ManagerLifecycleState::IDLE);
        }

        reset_fire_count();
        reset_control_outputs();

        auto input = input_context();
        auto output = output_context();
        auto manager_settings = settings();
        submit_task(make_belt_init_task(input, output, manager_settings));
        RCLCPP_INFO(logger_, "[DartManager] queued BeltInitTask for recovery");
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
            if (task_name == "fire_preload") {
                increment_fire_count();
            }
            log_carriage_calibration_encoder(task_name);
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

    void log_carriage_calibration_encoder(const std::string& task_name) {
        if (task_name == "carriage_init"
            && runtime_state_.carriage_power_cycle_origin_angle.has_value()) {
            RCLCPP_INFO(
                logger_, "[DartManager] carriage_init finalized encoder origin=%.6f",
                *runtime_state_.carriage_power_cycle_origin_angle);
        }
    }

    void on_task_failure() {
        if (!task_state_.current_task) {
            return;
        }

        task_state_.current_task->finish_failure();
        reset_task_state(task_state_);

        reset_control_outputs();

        transition_to(ManagerLifecycleState::ERROR);
    }

    void enter_belt_wait_zero_velocity_mode() {
        *belt_command_ = rmcs_msgs::DartMechanismCommand::WAIT;
        *belt_target_velocity_ = 0.0;
        *belt_exit_mode_ = rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY;
        *belt_max_torque_override_ = std::numeric_limits<double>::quiet_NaN();
    }

    void reset_control_outputs() {
        enter_belt_wait_zero_velocity_mode();
        *lift_command_ = rmcs_msgs::DartMechanismCommand::WAIT;
        *lift_target_velocity_ = 0.0;
        *lift_exit_mode_ = rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY;
        *trigger_command_ = rmcs_msgs::DartServoCommand::WAIT;
        *limiting_command_ = rmcs_msgs::DartServoCommand::LOCK;
        *carriage_command_ = rmcs_msgs::DartMechanismCommand::WAIT;
        *carriage_target_velocity_ = 0.0;
        *carriage_target_angle_ = std::numeric_limits<double>::quiet_NaN();
        *force_error_ = 0;
        *force_max_velocity_override_ = std::numeric_limits<double>::quiet_NaN();
        *force_max_torque_override_ = std::numeric_limits<double>::quiet_NaN();
        *angle_error_vector_ = Eigen::Vector2d::Zero();
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

    bool manual_control_active() const {
        return task_state_.current_task && task_state_.current_task->name() == "manual_control";
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
        *debug_manual_control_active_output_ = manual_control_active();
        *debug_queue_output_ = build_queue_snapshot();
        *debug_last_error_output_ = last_error_;
    }

    void reset_fire_count() { runtime_state_.fire_count = 0; }

    void increment_fire_count() {
        ++runtime_state_.fire_count;
        RCLCPP_INFO(logger_, "[DartManager] fire_count=%u", runtime_state_.fire_count);
    }

    ManagerInputContext input_context() {
        return ManagerInputContext{
            *belt_left_angle_,               //
            *belt_left_velocity_,            //
            *belt_left_torque_,              //
            *belt_right_angle_,              //
            *belt_right_velocity_,           //
            *belt_right_torque_,             //
            *lift_left_velocity_,            //
            *lift_left_torque_,              //
            *lift_right_velocity_,           //
            *lift_right_torque_,             //
            *force_screw_encoder_angle_,     //
            *force_screw_velocity_,          //
            *force_screw_torque_,            //
            *carriage_origin_angle_,         //
            *leveling_front_left_velocity_,  //
            *leveling_front_left_torque_,    //
            *leveling_front_right_velocity_, //
            *leveling_front_right_torque_,   //
            *leveling_rear_left_velocity_,   //
            *leveling_rear_left_torque_,     //
            *leveling_rear_right_velocity_,  //
            *leveling_rear_right_torque_,    //
            *force_sensor_ch1_,              //
            *force_sensor_ch2_,              //
            *current_target_input_,          //
            *tracking_input_,                //
            *target_seq_input_,              //
            *pitch_angle_,                   //
            *roll_angle_,                    //
            *remote_left_switch_,            //
            *remote_right_switch_,           //
            *remote_rotary_knob_switch_,     //
            *remote_left_joystick_,          //
            *remote_right_joystick_,         //
        };
    }

    ManagerOutputContext output_context() {
        return ManagerOutputContext{
            *belt_command_,                  //
            *belt_target_velocity_,          //
            *belt_exit_mode_,                //
            *belt_max_torque_override_,      //
            *lift_command_,                  //
            *lift_target_velocity_,          //
            *lift_exit_mode_,                //
            *trigger_command_,               //
            *limiting_command_,              //
            *carriage_command_,              //
            *carriage_target_velocity_,      //
            *carriage_target_angle_,         //
            *carriage_origin_angle_,         //
            *chassis_pitch_leveling_flag_,   //
            *chassis_roll_leveling_flag_,    //
            *force_error_,                   //
            *force_max_velocity_override_,   //
            *force_max_torque_override_,     //
            *angle_error_vector_,            //
        };
    }

    ManagerSettings settings() const {
        return ManagerSettings{
            belt_down_velocity_,             //
            belt_down_travel_angle_,         //
            belt_up_velocity_,               //
            belt_up_travel_angle_,           //
            belt_interference_relief_travel_angle_,
            belt_init_velocity_,             //
            belt_stall_velocity_threshold_,  //
            belt_stall_torque_threshold_,    //
            belt_stall_confirm_ticks_,       //
            belt_init_stall_velocity_threshold_,
            belt_init_stall_torque_threshold_,
            belt_init_stall_confirm_ticks_,
            belt_init_max_torque_,
            manual_belt_velocity_,           //
            lift_velocity_,                  //
            lift_stall_velocity_threshold_,  //
            lift_stall_torque_threshold_,    //
            lift_stall_confirm_ticks_,       //
            carriage_down_velocity_,
            carriage_travel_angle_,
            carriage_up_velocity_,
            carriage_lift_down_limit_,
            carriage_adjust_down_angle_,
            carriage_adjust_up_angle_,
            carriage_stall_velocity_threshold_,
            carriage_stall_torque_threshold_,
            carriage_stall_confirm_ticks_,
            carriage_calibration_velocity_,
            carriage_calibration_stall_velocity_threshold_,
            carriage_calibration_stall_torque_threshold_,
            carriage_calibration_stall_confirm_ticks_,
            carriage_calibration_max_torque_,
            carriage_calibration_parking_angle_,
            carriage_angle_allowable_error_,
            carriage_min_run_ticks_,
            carriage_timeout_ticks_,
            limiting_fill_ticks_,            //
            manual_angle_max_error_,         //
            manual_force_max_error_,         //
        };
    }

    rclcpp::Logger logger_;

    // belt
    OutputInterface<rmcs_msgs::DartMechanismCommand> belt_command_;
    OutputInterface<double> belt_target_velocity_;
    OutputInterface<rmcs_msgs::ExitMode> belt_exit_mode_;
    OutputInterface<double> belt_max_torque_override_;
    InputInterface<double> belt_left_angle_;
    InputInterface<double> belt_left_velocity_;
    InputInterface<double> belt_left_torque_;
    InputInterface<double> belt_right_angle_;
    InputInterface<double> belt_right_velocity_;
    InputInterface<double> belt_right_torque_;

    double belt_down_velocity_;
    double belt_down_travel_angle_;
    double belt_up_velocity_;
    double belt_up_travel_angle_;
    double belt_interference_relief_travel_angle_;
    double belt_init_velocity_;
    double belt_stall_velocity_threshold_;
    double belt_stall_torque_threshold_;
    uint64_t belt_stall_confirm_ticks_;
    double belt_init_stall_velocity_threshold_;
    double belt_init_stall_torque_threshold_;
    uint64_t belt_init_stall_confirm_ticks_;
    double belt_init_max_torque_;

    // lift
    OutputInterface<rmcs_msgs::DartMechanismCommand> lift_command_;
    OutputInterface<double> lift_target_velocity_;
    OutputInterface<rmcs_msgs::ExitMode> lift_exit_mode_;
    InputInterface<double> lift_left_velocity_;
    InputInterface<double> lift_left_torque_;
    InputInterface<double> lift_right_velocity_;
    InputInterface<double> lift_right_torque_;

    double lift_velocity_;
    double lift_stall_velocity_threshold_;
    double lift_stall_torque_threshold_;
    uint64_t lift_stall_confirm_ticks_;
    uint64_t lift_stall_min_run_ticks_;

    // carriage
    OutputInterface<rmcs_msgs::DartMechanismCommand> carriage_command_;
    OutputInterface<double> carriage_target_velocity_;
    OutputInterface<double> carriage_target_angle_;
    OutputInterface<double> carriage_origin_angle_;
    InputInterface<double> force_screw_angle_;
    InputInterface<double> force_screw_encoder_angle_;
    InputInterface<double> force_screw_velocity_;
    InputInterface<double> force_screw_torque_;
    std::string fire_target_;
    double basement_travel_angle_;
    double frontier_travel_angle_;
    double carriage_down_velocity_;
    double carriage_up_velocity_;
    double carriage_lift_down_limit_;
    double carriage_startup_position_angle_;
    bool carriage_startup_position_initialized_{false};
    double carriage_travel_angle_;
    double carriage_adjust_down_angle_;
    double carriage_adjust_up_angle_;
    double carriage_stall_velocity_threshold_;
    double carriage_stall_torque_threshold_;
    uint64_t carriage_stall_confirm_ticks_;
    double carriage_calibration_velocity_;
    double carriage_calibration_stall_velocity_threshold_;
    double carriage_calibration_stall_torque_threshold_;
    uint64_t carriage_calibration_stall_confirm_ticks_;
    double carriage_calibration_max_torque_;
    double carriage_calibration_parking_angle_;
    double carriage_angle_allowable_error_;
    uint64_t carriage_min_run_ticks_;
    uint64_t carriage_timeout_ticks_;

    // trigger
    OutputInterface<rmcs_msgs::DartServoCommand> trigger_command_;

    // limit servo
    OutputInterface<rmcs_msgs::DartServoCommand> limiting_command_;
    uint64_t limiting_fill_ticks_;

    // chassis leveling
    OutputInterface<bool> chassis_pitch_leveling_flag_;
    OutputInterface<bool> chassis_roll_leveling_flag_;
    InputInterface<double> leveling_front_left_velocity_;
    InputInterface<double> leveling_front_left_torque_;
    InputInterface<double> leveling_front_right_velocity_;
    InputInterface<double> leveling_front_right_torque_;
    InputInterface<double> leveling_rear_left_velocity_;
    InputInterface<double> leveling_rear_left_torque_;
    InputInterface<double> leveling_rear_right_velocity_;
    InputInterface<double> leveling_rear_right_torque_;

    // yaw pitch force
    OutputInterface<int32_t> force_error_;
    OutputInterface<double> force_max_velocity_override_;
    OutputInterface<double> force_max_torque_override_;
    OutputInterface<Eigen::Vector2d> angle_error_vector_;

    InputInterface<int32_t> force_sensor_ch1_;
    InputInterface<int32_t> force_sensor_ch2_;

    InputInterface<cv::Point2i> current_target_input_;
    InputInterface<bool> tracking_input_;
    InputInterface<uint64_t> target_seq_input_;
    InputInterface<double> pitch_angle_;
    InputInterface<double> roll_angle_;

    // manual control
    InputInterface<rmcs_msgs::Switch> remote_left_switch_;
    InputInterface<rmcs_msgs::Switch> remote_right_switch_;
    InputInterface<rmcs_msgs::Switch> remote_rotary_knob_switch_;
    InputInterface<Eigen::Vector2d> remote_left_joystick_;
    InputInterface<Eigen::Vector2d> remote_right_joystick_;

    double manual_belt_velocity_;
    int32_t manual_force_max_error_;
    double manual_angle_max_error_;

    // command & status
    InputInterface<std::string> command_input_;

    OutputInterface<uint32_t> fire_count_output_;
    OutputInterface<std::string> debug_lifecycle_state_output_;
    OutputInterface<std::string> debug_current_task_output_;
    OutputInterface<std::string> debug_current_action_output_;
    OutputInterface<bool> debug_manual_control_active_output_;
    OutputInterface<std::vector<ManagerQueuedTaskInfo>> debug_queue_output_;
    OutputInterface<std::optional<ManagerLastErrorInfo>> debug_last_error_output_;

    std::optional<ManagerLastErrorInfo> last_error_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr carriage_position_calibrate_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr carriage_adjust_down_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr carriage_adjust_up_subscription_;
    VisionAimProfileProvider vision_aim_profile_provider_;

    ManagerRuntimeState runtime_state_{};
    TaskState task_state_{};

    int log_counter_ = 0;
};

} // namespace rmcs_dart_guidance::manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::manager::DartManager, rmcs_executor::Component)
