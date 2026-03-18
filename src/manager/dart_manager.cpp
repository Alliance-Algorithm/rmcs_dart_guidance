#include "manager/action/action.hpp"
#include "manager/action/manual_angle_control.hpp"
#include "manager/action/manual_force_control.hpp"
#include "manager/auto_aim_feedback.hpp"
#include "manager/dart_launch_sequence.hpp"
#include "manager/task/cancel_launch_task.hpp"
#include "manager/task/fire_and_preload_task.hpp"
#include "manager/task/launch_preparation_task.hpp"
#include "manager/task/silder_init_task.hpp"
#include "manager/task/task.hpp"
#include "manager/task/vision_assisted_launch_preparation_task.hpp"

#include <algorithm>
#include <cstdint>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <rmcs_msgs/dart_limiting_servo_status.hpp>
#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

// DartManagerV2
//   · /dart/manager/lifting/command、/dart/manager/limiting/command 输出给下层执行组件
//   · 升降堵转检测仍在 FillingLiftAction 内完成（直接读速度反馈，无循环依赖）
class DartManagerV2
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    enum class State : uint8_t {
        IDLE    = 0,
        RUNNING = 1,
        ERROR   = 2,
    };

    DartManagerV2()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        register_input("/dart/drive_belt/left/velocity",  left_belt_velocity_);
        register_input("/dart/drive_belt/right/velocity", right_belt_velocity_);
        register_input("/dart/drive_belt/left/torque",   left_belt_torque_);
        register_input("/dart/drive_belt/right/torque",  right_belt_torque_);
        register_input("/dart/lifting_left/velocity",    lifting_left_vel_fb_);
        register_input("/dart/lifting_right/velocity",   lifting_right_vel_fb_);

        register_input("/dart/manager/command",     remote_command_input_, false);
        register_input("/dart/manager/web_command", web_command_input_,    false);

        register_input("/remote/joystick/left",  joystick_left_,  false);
        register_input("/remote/joystick/right", joystick_right_, false);
        register_input("/dart_guidance/camera/target_position", target_position_input_, false);
        register_input("/dart_guidance/tracker/tracking", target_tracking_input_, false);

        register_output("/dart/manager/belt/command", belt_command_, rmcs_msgs::DartSliderStatus::WAIT);
        register_output("/dart/manager/belt/target_velocity", belt_target_velocity_, 0.0);
        register_output("/dart/manager/belt/torque_limit", belt_torque_limit_, 0.0);
        register_output("/dart/manager/belt/hold_torque", belt_hold_torque_, 0.0);
        register_output("/dart/manager/belt/wait_zero_velocity", belt_wait_zero_velocity_, false);
        register_output("/dart/manager/trigger/lock_enable", trigger_lock_enable_, false);

        register_output("/pitch/control/velocity", yaw_pitch_control_velocity_, Eigen::Vector2d::Zero());
        register_output("/force/control/velocity", force_control_velocity_, 0.0);
        register_output("/dart/manager/aim/ready", aim_ready_, false);
        register_output(
            "/dart/manager/aim/current_dart_index", aim_current_dart_index_,
            static_cast<uint8_t>(0));
        register_output("/dart/manager/aim/error_px", aim_error_px_, Eigen::Vector2d::Zero());
        register_output(
            "/dart/manager/aim/desired_target_px", aim_desired_target_px_, Eigen::Vector2d::Zero());
        auto_aim_feedback_.bind(
            *yaw_pitch_control_velocity_, *aim_ready_, *aim_error_px_, *aim_desired_target_px_);

        // 升降指令总线
        register_output(
            "/dart/manager/lifting/command", lifting_command_,
            rmcs_msgs::DartSliderStatus::WAIT);
        register_output(
            "/dart/manager/limiting/command", limiting_command_,
            rmcs_msgs::DartLimitingServoStatus::LOCK);

        try {
            max_transform_rate_ = get_parameter("max_transform_rate").as_double();
        } catch (...) {
            max_transform_rate_ = 500.0;
        }
        try {
            manual_force_scale_ = get_parameter("manual_force_scale").as_double();
        } catch (...) {
            manual_force_scale_ = 5.0;
        }
        launch_prepare_enable_visual_assist_ =
            has_parameter("launch_prepare_enable_visual_assist")
                ? get_parameter("launch_prepare_enable_visual_assist").as_bool()
                : false;
        if (launch_prepare_enable_visual_assist_) {
            load_auto_aim_configuration();
        }

        limiting_fill_ticks_  = (uint64_t)get_parameter("limiting_fill_ticks").as_int();

        lifting_stall_threshold_    = get_parameter("lifting_stall_threshold").as_double();
        lifting_stall_confirm_ticks_ =
            (uint64_t)get_parameter("lifting_stall_confirm_ticks").as_int();
        lifting_stall_min_run_ticks_ =
            (uint64_t)get_parameter("lifting_stall_min_run_ticks").as_int();
        lifting_stall_timeout_ticks_ =
            (uint64_t)get_parameter("lifting_stall_timeout_ticks").as_int();

        state_pub_ = create_publisher<std_msgs::msg::UInt8>("/dart/manager/state", 10);

        sync_current_dart_outputs();
        clear_auto_aim_feedback();
        submit_task(make_slider_init_task());
        RCLCPP_INFO(logger_, "[DartManagerV2] initialized, queued SliderInitTask on startup");
    }

    void update() override {
        refresh_auto_aim_inputs();
        poll_command();

        switch (state_) {
        case State::IDLE:    dispatch_next_task(); break;
        case State::RUNNING: tick_current_task();  break;
        case State::ERROR:   break;
        }
    }

private:
    void poll_command() {
        std::string cmd;

        if (web_command_input_.ready() && !web_command_input_->empty()) {
            cmd = *web_command_input_;
        } else if (remote_command_input_.ready()) {
            cmd = *remote_command_input_;
        }

        if (cmd.empty()) {
            last_command_.clear();
            return;
        }

        if (cmd == last_command_)
            return;

        last_command_ = cmd;

        if (cmd == "cancel") {
            cancel_all();
        } else if (cmd == "recover") {
            recover();
        } else {
            auto task = make_task(cmd);
            RCLCPP_INFO(logger_, "[DartManagerV2] received command: '%s'", cmd.c_str());
            if (task) {
                submit_task(std::move(task));
            } else {
                RCLCPP_WARN(logger_, "[DartManagerV2] unknown command: '%s'", cmd.c_str());
            }
        }
    }

    void submit_task(std::shared_ptr<Task> task) {
        task_queue_.push_back(std::move(task));
        RCLCPP_INFO(
            logger_, "[DartManagerV2] task queued: %s (queue size=%zu)",
            task_queue_.back()->name().c_str(), task_queue_.size());
    }

    void cancel_all() {
        task_queue_.clear();
        if (current_task_) {
            current_task_->cancel();
            current_task_.reset();
            RCLCPP_WARN(logger_, "[DartManagerV2] all tasks cancelled");
        }

        first_fill_pending_ = true;

        enter_belt_wait_zero_velocity_mode();
        *lifting_command_ = rmcs_msgs::DartSliderStatus::WAIT;
        *limiting_command_ = rmcs_msgs::DartLimitingServoStatus::LOCK;
        *yaw_pitch_control_velocity_ = Eigen::Vector2d::Zero();
        *force_control_velocity_ = 0.0;
        clear_auto_aim_feedback();

        transition_to(State::IDLE);
    }

    void recover() {
        if (state_ == State::ERROR) {
            current_task_.reset();
            task_queue_.clear();
            RCLCPP_INFO(logger_, "[DartManagerV2] recovered from ERROR, state=IDLE");
            transition_to(State::IDLE);
        }

        first_fill_pending_ = true;
        *limiting_command_ = rmcs_msgs::DartLimitingServoStatus::LOCK;
        if (launch_prepare_enable_visual_assist_) {
            reset_dart_sequence();
        }
        clear_auto_aim_feedback();

        // 无论 ERROR 还是 IDLE，都重新排队传送带复位
        submit_task(make_slider_init_task());
        RCLCPP_INFO(logger_, "[DartManagerV2] queued SliderInitTask for recovery");
    }

    void dispatch_next_task() {
        if (task_queue_.empty())
            return;

        current_task_ = std::move(task_queue_.front());
        task_queue_.pop_front();
        prepare_outputs_for_task(*current_task_);

        RCLCPP_INFO(
            logger_, "[DartManagerV2] dispatching task: '%s'", current_task_->name().c_str());
        transition_to(State::RUNNING);

        tick_current_task();
    }

    void tick_current_task() {
        if (!current_task_)
            return;

        ActionStatus status =
            first_tick_of_task_ ? current_task_->tick_first() : current_task_->tick();
        first_tick_of_task_ = false;

        if (status == ActionStatus::SUCCESS) {
            const std::string completed_task_name = current_task_->name();
            current_task_->on_exit();
            RCLCPP_INFO(
                logger_, "[DartManagerV2] task '%s' SUCCESS", completed_task_name.c_str());
            if (launch_prepare_enable_visual_assist_ && completed_task_name == "fire") {
                advance_dart_sequence_after_fire();
            }
            current_task_.reset();
            transition_to(State::IDLE);

        } else if (status == ActionStatus::FAILURE) {
            RCLCPP_ERROR(
                logger_, "[DartManagerV2] task '%s' FAILURE → state=ERROR",
                current_task_->name().c_str());
            on_task_failure();
        }
    }

    void on_task_failure() {
        current_task_->on_exit();
        current_task_.reset();
        task_queue_.clear();

        enter_belt_wait_zero_velocity_mode();
        *lifting_command_ = rmcs_msgs::DartSliderStatus::WAIT;
        *limiting_command_ = rmcs_msgs::DartLimitingServoStatus::LOCK;
        *yaw_pitch_control_velocity_ = Eigen::Vector2d::Zero();
        *force_control_velocity_ = 0.0;
        clear_auto_aim_feedback();

        transition_to(State::ERROR);
    }

    void enter_belt_wait_zero_velocity_mode() {
        *belt_command_ = rmcs_msgs::DartSliderStatus::WAIT;
        *belt_target_velocity_ = 0.0;
        *belt_hold_torque_ = 0.0;
        *belt_wait_zero_velocity_ = true;
    }

    void transition_to(State new_state) {
        state_             = new_state;
        first_tick_of_task_ = true;

        if (state_pub_) {
            std_msgs::msg::UInt8 msg;
            msg.data = static_cast<uint8_t>(new_state);
            state_pub_->publish(msg);
        }
    }

    void refresh_auto_aim_inputs() {
        if (target_position_input_.ready()) {
            current_target_position_ = *target_position_input_;
        } else {
            current_target_position_ = cv::Point2i(-1, -1);
        }

        current_target_tracking_ =
            target_tracking_input_.ready() ? *target_tracking_input_ : false;
    }

    double get_numeric_parameter_or_throw(const std::string& name) const {
        if (!has_parameter(name)) {
            throw std::runtime_error("Missing required parameter: " + name);
        }

        const auto parameter = get_parameter(name);
        switch (parameter.get_type()) {
        case rclcpp::PARAMETER_DOUBLE: return parameter.as_double();
        case rclcpp::PARAMETER_INTEGER: return static_cast<double>(parameter.as_int());
        default: throw std::runtime_error("Parameter must be numeric: " + name);
        }
    }

    uint64_t get_uint_parameter_or_throw(const std::string& name) const {
        if (!has_parameter(name)) {
            throw std::runtime_error("Missing required parameter: " + name);
        }

        const auto value = get_parameter(name).as_int();
        if (value <= 0) {
            throw std::runtime_error("Parameter must be positive: " + name);
        }
        return static_cast<uint64_t>(value);
    }

    std::vector<double> get_numeric_array_parameter_or_throw(const std::string& name) const {
        if (!has_parameter(name)) {
            throw std::runtime_error("Missing required parameter: " + name);
        }

        const auto parameter = get_parameter(name);
        switch (parameter.get_type()) {
        case rclcpp::PARAMETER_DOUBLE_ARRAY: return parameter.as_double_array();
        case rclcpp::PARAMETER_INTEGER_ARRAY: {
            const auto values = parameter.as_integer_array();
            std::vector<double> result;
            result.reserve(values.size());
            for (const auto value : values) {
                result.push_back(static_cast<double>(value));
            }
            return result;
        }
        default: throw std::runtime_error("Parameter must be a numeric array: " + name);
        }
    }

    Eigen::Vector2d get_vector2_parameter_or_throw(const std::string& name) const {
        const auto values = get_numeric_array_parameter_or_throw(name);
        if (values.size() != 2) {
            throw std::runtime_error("Parameter must contain exactly 2 values: " + name);
        }
        return Eigen::Vector2d(values[0], values[1]);
    }

    void load_auto_aim_configuration() {
        int64_t dart_count = 4;
        if (has_parameter("dart_count")) {
            dart_count = get_parameter("dart_count").as_int();
        }

        aim_deadband_px_ = has_parameter("aim_deadband_px")
                             ? get_vector2_parameter_or_throw("aim_deadband_px")
                             : Eigen::Vector2d::Constant(3.0);
        aim_ready_exit_deadband_px_ =
            has_parameter("aim_ready_exit_deadband_px")
                ? get_vector2_parameter_or_throw("aim_ready_exit_deadband_px")
                : Eigen::Vector2d::Constant(5.0);
        aim_accept_deadband_px_ =
            has_parameter("aim_accept_deadband_px")
                ? get_vector2_parameter_or_throw("aim_accept_deadband_px")
                : aim_ready_exit_deadband_px_;
        aim_yaw_gain_ = get_numeric_parameter_or_throw("aim_yaw_gain");
        aim_pitch_gain_ = get_numeric_parameter_or_throw("aim_pitch_gain");
        aim_ready_confirm_ticks_ = has_parameter("aim_ready_confirm_ticks")
                                     ? get_uint_parameter_or_throw("aim_ready_confirm_ticks")
                                     : 5;
        aim_timeout_ticks_ = get_uint_parameter_or_throw("aim_timeout_ticks");
        aim_min_transform_rate_ = has_parameter("aim_min_transform_rate")
                                    ? get_numeric_parameter_or_throw("aim_min_transform_rate")
                                    : 0.0;
        auto_aim_max_transform_rate_ =
            has_parameter("auto_aim_max_transform_rate")
                ? get_numeric_parameter_or_throw("auto_aim_max_transform_rate")
                : max_transform_rate_;

        if (aim_deadband_px_.x() < 0.0 || aim_deadband_px_.y() < 0.0) {
            throw std::runtime_error("Parameter aim_deadband_px must be non-negative");
        }
        if (aim_ready_exit_deadband_px_.x() < aim_deadband_px_.x()
            || aim_ready_exit_deadband_px_.y() < aim_deadband_px_.y()) {
            throw std::runtime_error(
                "Parameter aim_ready_exit_deadband_px must be >= aim_deadband_px");
        }
        if (aim_accept_deadband_px_.x() < aim_ready_exit_deadband_px_.x()
            || aim_accept_deadband_px_.y() < aim_ready_exit_deadband_px_.y()) {
            throw std::runtime_error(
                "Parameter aim_accept_deadband_px must be >= aim_ready_exit_deadband_px");
        }
        if (aim_min_transform_rate_ < 0.0) {
            throw std::runtime_error("Parameter aim_min_transform_rate must be non-negative");
        }
        if (auto_aim_max_transform_rate_ < 0.0) {
            throw std::runtime_error("Parameter auto_aim_max_transform_rate must be non-negative");
        }
        if (aim_min_transform_rate_ > auto_aim_max_transform_rate_) {
            throw std::runtime_error(
                "Parameter aim_min_transform_rate must be <= auto_aim_max_transform_rate");
        }

        dart_launch_sequence_.configure_from_parameter_values(
            DartLaunchSequenceRawConfig{
                .dart_count = dart_count,
                .aim_reference_pixel = get_numeric_array_parameter_or_throw("aim_reference_pixel"),
                .aim_dart_offsets_px = get_numeric_array_parameter_or_throw("aim_dart_offsets_px"),
            });
    }

    Eigen::Vector2d current_desired_target_px() const {
        return launch_prepare_enable_visual_assist_
                 ? dart_launch_sequence_.current_desired_target_px()
                 : Eigen::Vector2d::Zero();
    }

    void sync_current_dart_outputs() {
        *aim_current_dart_index_ =
            launch_prepare_enable_visual_assist_ ? dart_launch_sequence_.current_dart_index_u8()
                                                 : static_cast<uint8_t>(0);
        auto_aim_feedback_.set_desired_target_px(current_desired_target_px());
    }

    void clear_auto_aim_feedback() {
        *aim_current_dart_index_ =
            launch_prepare_enable_visual_assist_ ? dart_launch_sequence_.current_dart_index_u8()
                                                 : static_cast<uint8_t>(0);
        auto_aim_feedback_.reset(current_desired_target_px());
    }

    void reset_dart_sequence() {
        if (launch_prepare_enable_visual_assist_) {
            dart_launch_sequence_.reset();
        }
        sync_current_dart_outputs();
    }

    void advance_dart_sequence_after_fire() {
        if (!dart_launch_sequence_.advance_after_fire()) {
            RCLCPP_WARN(logger_, "[DartManagerV2] current dart index already at the last dart");
        }
        sync_current_dart_outputs();
        clear_auto_aim_feedback();
    }

    void prepare_outputs_for_task(const Task& task) {
        if (
            task.name() == "launch_preparation" || task.name() == "fire"
            || task.name() == "cancel_launch" || task.name() == "slider_init"
            || task.name() == "manual_angle" || task.name() == "manual_force") {
            clear_auto_aim_feedback();
        }
    }

    std::shared_ptr<Task> make_slider_init_task() {
        return std::make_shared<SliderInitTask>(
            *belt_command_,
            *belt_target_velocity_, *belt_torque_limit_, *belt_hold_torque_,
            *belt_wait_zero_velocity_,
            *left_belt_velocity_,    *right_belt_velocity_,
            *left_belt_torque_,      *right_belt_torque_);
    }

    // 任务工厂
    std::shared_ptr<Task> make_task(const std::string& cmd) {
        if (cmd == "launch_prepare" || cmd == "launch-prepare") {
            auto launch_mode = first_fill_pending_
                             ? LaunchPreparationTask::Mode::FIRST_FILL
                             : LaunchPreparationTask::Mode::NORMAL;
            first_fill_pending_ = false;

            if (launch_prepare_enable_visual_assist_) {
                return std::make_shared<VisionAssistedLaunchPreparationTask>(
                    *belt_command_, *belt_target_velocity_, *belt_torque_limit_, *belt_hold_torque_,
                    *belt_wait_zero_velocity_, *left_belt_velocity_, *right_belt_velocity_,
                    *left_belt_torque_, *right_belt_torque_, *trigger_lock_enable_,
                    *lifting_command_, *lifting_left_vel_fb_, *lifting_right_vel_fb_,
                    lifting_stall_threshold_, lifting_stall_confirm_ticks_,
                    lifting_stall_min_run_ticks_, lifting_stall_timeout_ticks_, launch_mode,
                    auto_aim_feedback_.yaw_pitch_control_velocity(),
                    auto_aim_feedback_.aim_ready(), auto_aim_feedback_.aim_error_px(),
                    auto_aim_feedback_.desired_target_px(), current_target_position_,
                    current_target_tracking_, logger_,
                    AutoAimParams{
                        .desired_target_px = current_desired_target_px(),
                        .deadband_px = aim_deadband_px_,
                        .ready_exit_deadband_px = aim_ready_exit_deadband_px_,
                        .accept_deadband_px = aim_accept_deadband_px_,
                        .yaw_gain = aim_yaw_gain_,
                        .pitch_gain = aim_pitch_gain_,
                        .ready_confirm_ticks = aim_ready_confirm_ticks_,
                        .timeout_ticks = aim_timeout_ticks_,
                        .min_transform_rate = aim_min_transform_rate_,
                        .max_transform_rate = auto_aim_max_transform_rate_,
                    });
            }

            return std::make_shared<LaunchPreparationTask>(
                *belt_command_, *belt_target_velocity_, *belt_torque_limit_, *belt_hold_torque_,
                *belt_wait_zero_velocity_, *left_belt_velocity_, *right_belt_velocity_,
                *left_belt_torque_, *right_belt_torque_, *trigger_lock_enable_,
                *lifting_command_, *lifting_left_vel_fb_, *lifting_right_vel_fb_,
                lifting_stall_threshold_, lifting_stall_confirm_ticks_,
                lifting_stall_min_run_ticks_, lifting_stall_timeout_ticks_, launch_mode);
        }

        if (cmd == "unload" || cmd == "cancel_launch") {
            first_fill_pending_ = true;

            return std::make_shared<CancelLaunchTask>(
                *belt_command_, *belt_target_velocity_, *belt_torque_limit_, *belt_hold_torque_,
                *belt_wait_zero_velocity_,
                *left_belt_velocity_, *right_belt_velocity_,
                *left_belt_torque_,   *right_belt_torque_,
                *trigger_lock_enable_,
                *lifting_command_,
                *lifting_left_vel_fb_, *lifting_right_vel_fb_,
                lifting_stall_threshold_, lifting_stall_confirm_ticks_,
                lifting_stall_min_run_ticks_, lifting_stall_timeout_ticks_);
        }

        if (cmd == "fire") {
            return std::make_shared<FireAndPreloadTask>(
                *trigger_lock_enable_,
                *lifting_command_,
                *lifting_left_vel_fb_, *lifting_right_vel_fb_,
                lifting_stall_threshold_, lifting_stall_confirm_ticks_,
                lifting_stall_min_run_ticks_, lifting_stall_timeout_ticks_,
                *limiting_command_, limiting_fill_ticks_);
        }

        if (cmd == "manual_angle") {
            auto task = std::make_shared<Task>("manual_angle", "手动 yaw/pitch 调整");
            task->then(std::make_shared<DartManualAngleControlAction>(
                auto_aim_feedback_.yaw_pitch_control_velocity()[0],
                auto_aim_feedback_.yaw_pitch_control_velocity()[1], *joystick_left_,
                *joystick_right_, max_transform_rate_));
            return task;
        }

        if (cmd == "manual_force") {
            auto task = std::make_shared<Task>("manual_force", "手动力丝杆速度调整");
            task->then(std::make_shared<DartManualForceControlAction>(
                *force_control_velocity_,
                *joystick_right_,
                max_transform_rate_,
                manual_force_scale_));
            return task;
        }
        return nullptr;
    }

    rclcpp::Logger logger_;

    InputInterface<double> left_belt_velocity_;
    InputInterface<double> right_belt_velocity_;
    InputInterface<double> left_belt_torque_;
    InputInterface<double> right_belt_torque_;

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<cv::Point2i>     target_position_input_;
    InputInterface<bool>            target_tracking_input_;

    // 升降速度反馈（FillingLiftAction 堵转检测用）
    InputInterface<double> lifting_left_vel_fb_;
    InputInterface<double> lifting_right_vel_fb_;

    OutputInterface<rmcs_msgs::DartSliderStatus> belt_command_;
    OutputInterface<double> belt_target_velocity_;
    OutputInterface<double> belt_torque_limit_;
    OutputInterface<double> belt_hold_torque_;
    OutputInterface<bool>   belt_wait_zero_velocity_;
    OutputInterface<bool>   trigger_lock_enable_;

    OutputInterface<Eigen::Vector2d> yaw_pitch_control_velocity_;
    OutputInterface<double>          force_control_velocity_;
    OutputInterface<bool>            aim_ready_;
    OutputInterface<uint8_t>         aim_current_dart_index_;
    OutputInterface<Eigen::Vector2d> aim_error_px_;
    OutputInterface<Eigen::Vector2d> aim_desired_target_px_;

    OutputInterface<rmcs_msgs::DartSliderStatus> lifting_command_;
    OutputInterface<rmcs_msgs::DartLimitingServoStatus> limiting_command_;

    double   max_transform_rate_{500.0};
    double   manual_force_scale_{5.0};
    double   auto_aim_max_transform_rate_{500.0};
    uint64_t limiting_fill_ticks_{500};

    double   lifting_stall_threshold_{0.5};
    uint64_t lifting_stall_confirm_ticks_{100};
    uint64_t lifting_stall_min_run_ticks_{500};
    uint64_t lifting_stall_timeout_ticks_{5000};
    bool launch_prepare_enable_visual_assist_{false};
    AutoAimFeedback auto_aim_feedback_;
    DartLaunchSequence dart_launch_sequence_;
    Eigen::Vector2d aim_deadband_px_{Eigen::Vector2d::Constant(3.0)};
    Eigen::Vector2d aim_ready_exit_deadband_px_{Eigen::Vector2d::Constant(5.0)};
    Eigen::Vector2d aim_accept_deadband_px_{Eigen::Vector2d::Constant(8.0)};
    double aim_yaw_gain_{0.0};
    double aim_pitch_gain_{0.0};
    uint64_t aim_ready_confirm_ticks_{5};
    uint64_t aim_timeout_ticks_{3000};
    double aim_min_transform_rate_{0.0};
    cv::Point2i current_target_position_{-1, -1};
    bool current_target_tracking_{false};

    InputInterface<std::string> remote_command_input_;
    InputInterface<std::string> web_command_input_;
    std::string                 last_command_;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;

    State state_{State::IDLE};

    std::shared_ptr<Task>             current_task_;
    std::deque<std::shared_ptr<Task>> task_queue_;
    bool first_fill_pending_{true};
    bool first_tick_of_task_{true};
};

} // namespace rmcs_dart_guidance::manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::manager::DartManagerV2, rmcs_executor::Component)
