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

// DartManager
//   · /dart/manager/lifting/command、/dart/manager/limiting/command 输出给下层执行组件
//   · 升降堵转检测仍在 FillingLiftAction 内完成（直接读速度反馈，无循环依赖）
class DartManager
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    enum class State : uint8_t {
        IDLE = 0,
        RUNNING = 1,
        ERROR = 2,
    };

    DartManager()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        register_input("/dart/drive_belt/left/velocity", left_belt_velocity_);
        register_input("/dart/drive_belt/right/velocity", right_belt_velocity_);
        register_input("/dart/drive_belt/left/torque", left_belt_torque_);
        register_input("/dart/drive_belt/right/torque", right_belt_torque_);
        register_input("/dart/drive_belt/left/angle", left_belt_angle_);
        register_input("/dart/drive_belt/right/angle", right_belt_angle_);
        register_input("/dart/lifting_left/velocity", lifting_left_vel_fb_);
        register_input("/dart/lifting_right/velocity", lifting_right_vel_fb_);
        register_input("/dart/force_screw_motor/velocity", force_screw_velocity_fb_);
        register_input("/dart/force_screw_motor/torque", force_screw_torque_fb_);
        register_input("/force_sensor/channel_1/weight", current_force_ch1_);
        register_input("/force_sensor/channel_2/weight", current_force_ch2_);

        // Kalman filter inputs (optional, for force calibration)
        register_input("/dart/kalman/filtered_force", kalman_filtered_force_, false);
        register_input("/dart/kalman/force_rate", kalman_force_rate_, false);

        // register_input("/dart_guidance/camera/target_position", target_position_);

        register_input("/dart/manager/command", remote_command_input_);
        register_input("/dart/manager/web_command", web_command_input_);

        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/joystick/right", joystick_right_);
        // register_input("/dart_guidance/tracker/tracking", target_tracking_input_);
        // register_input("/dart_guidance/tracker/yaw_pitch_target_distance",
        // target_position_input_);

        register_output(
            "/dart/manager/belt/command", belt_command_, rmcs_msgs::DartSliderStatus::WAIT);
        register_output("/dart/manager/belt/target_velocity", belt_target_velocity_, 0.0);
        register_output("/dart/manager/belt/torque_limit", belt_torque_limit_, 0.0);
        register_output("/dart/manager/belt/hold_torque", belt_hold_torque_, 0.0);
        register_output("/dart/manager/belt/torque_offset", belt_torque_offset_, 0.0);
        register_output("/dart/manager/belt/wait_zero_velocity", belt_wait_zero_velocity_, false);
        register_output("/dart/manager/belt/zero_calibration", belt_zero_calibration_, false);
        register_output("/dart/manager/belt/error_gain", belt_error_gain_, 1.0);
        register_output("/dart/manager/belt/use_decel_pid", belt_use_decel_pid_, false);
        register_output("/dart/manager/trigger/lock_enable", trigger_lock_enable_, false);

        register_output(
            "/pitch/control/velocity", yaw_pitch_control_velocity_, Eigen::Vector2d::Zero());
        register_output("/pitch/control/angle", yaw_pitch_control_angle_, Eigen::Vector2d::Zero());
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
            "/dart/manager/lifting/command", lifting_command_, rmcs_msgs::DartSliderStatus::WAIT);
        register_output(
            "/dart/manager/limiting/command", limiting_command_,
            rmcs_msgs::DartLimitingServoStatus::LOCK);

        // 力传感器记录触发信号（给ForceSensorRecorder组件使用）
        register_output("/dart/manager/fire_trigger", fire_trigger_, false);

        yaw_transform_rate_ = get_parameter("yaw_transform_rate").as_double();
        manual_force_scale_ = get_parameter("manual_force_scale").as_double();

        enable_pixel_to_angle_ = get_parameter("enable_pixel_to_angle").as_bool();

        launch_prepare_enable_visual_assist_ =
            get_parameter("launch_prepare_enable_visual_assist").as_bool();
        if (launch_prepare_enable_visual_assist_) {
            load_auto_aim_configuration();
        }
        belt_down_distance_ = get_parameter("belt_down_distance").as_double(); // m

        enable_force_calibration_ = get_parameter("enable_force_calibration").as_bool();
        force_tolerance_ = get_parameter("force_tolerance").as_double();
        force_settle_ticks_ = (uint64_t)get_parameter("force_settle_ticks").as_int();
        force_timeout_ticks_ = (uint64_t)get_parameter("force_timeout_ticks").as_int();
        force_kp_ = get_parameter("force_kp").as_double();
        force_ki_ = get_parameter("force_ki").as_double();
        force_kd_ = get_parameter("force_kd").as_double();

        force_channel_ = (int)get_parameter("force_channel").as_int();
        if (force_channel_ != 1 && force_channel_ != 2) {
            RCLCPP_WARN(
                get_logger(), "[DartManager] force_channel=%d invalid, defaulting to 1",
                force_channel_);
            force_channel_ = 1;
        }

        // Kalman filter force calibration mode
        use_kalman_force_ = get_parameter_or("use_kalman_force", false);
        kalman_rate_feedforward_ = get_parameter_or("kalman_rate_feedforward", false);
        kalman_rate_gain_ = get_parameter_or("kalman_rate_gain", 0.0);

        if (use_kalman_force_) {
            RCLCPP_INFO(
                get_logger(), "[DartManager] Kalman force mode enabled: rate_ff=%d, rate_gain=%.3f",
                kalman_rate_feedforward_, kalman_rate_gain_);
        }

        state_pub_ = create_publisher<std_msgs::msg::UInt8>("/dart/manager/state", 10);

        temporary_flag_ = true;
        sync_current_dart_outputs();
        clear_auto_aim_feedback();
        submit_task(make_slider_init_task());
        RCLCPP_INFO(logger_, "[DartManager] initialized, queued SliderInitTask on startup");
    }

    void update() override {
        // 重置fire触发信号（单次脉冲）
        *fire_trigger_ = false;

        if (temporary_flag_) {
            submit_task(make_slider_init_task());
        }
        temporary_flag_ = false;
        poll_command();

        switch (state_) {
        case State::IDLE: dispatch_next_task(); break;
        case State::RUNNING: tick_current_task(); break;
        case State::ERROR: break;
        }
    }

private:
    template <typename T>
    T get_parameter_or(const std::string& name, T default_value) {
        if (has_parameter(name)) {
            return get_parameter(name).get_value<T>();
        }
        return default_value;
    }

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
            RCLCPP_INFO(logger_, "[DartManager] received command: '%s'", cmd.c_str());
            if (task) {
                submit_task(std::move(task));
            } else {
                RCLCPP_WARN(logger_, "[DartManager] unknown command: '%s'", cmd.c_str());
            }
        }
    }

    void submit_task(std::shared_ptr<Task> task) {
        task_queue_.push_back(std::move(task));
        RCLCPP_INFO(
            logger_, "[DartManager] task queued: %s (queue size=%zu)",
            task_queue_.back()->name().c_str(), task_queue_.size());
    }

    void cancel_all() {
        task_queue_.clear();
        if (current_task_) {
            current_task_->cancel();
            current_task_.reset();
            RCLCPP_WARN(logger_, "[DartManager] all tasks cancelled");
        }

        first_fill_pending_ = true;
        fire_count_ = 0; // 重置开火次数

        *belt_command_ = rmcs_msgs::DartSliderStatus::WAIT;
        *belt_target_velocity_ = 0.0;
        *belt_torque_limit_ = 0.0;
        *belt_hold_torque_ = 0.0;
        *belt_torque_offset_ = 0.0;
        *belt_wait_zero_velocity_ = false;
        *belt_error_gain_ = 1.0;
        *belt_use_decel_pid_ = false;
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
            RCLCPP_INFO(logger_, "[DartManager] recovered from ERROR, state=IDLE");
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
        RCLCPP_INFO(logger_, "[DartManager] queued SliderInitTask for recovery");
    }

    void dispatch_next_task() {
        if (task_queue_.empty())
            return;

        current_task_ = std::move(task_queue_.front());
        task_queue_.pop_front();
        prepare_outputs_for_task(*current_task_);

        RCLCPP_INFO(logger_, "[DartManager] dispatching task: '%s'", current_task_->name().c_str());
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
            RCLCPP_INFO(logger_, "[DartManager] task '%s' SUCCESS", completed_task_name.c_str());

            // 处理 fire 任务完成
            if (completed_task_name == "fire") {
                fire_count_++;
                RCLCPP_INFO(logger_, "[DartManager] fire completed, fire_count=%u", fire_count_);
                if (launch_prepare_enable_visual_assist_) {
                    advance_dart_sequence_after_fire();
                }
            }

            // 处理 cancel_launch 任务完成
            if (completed_task_name == "cancel_launch") {
                RCLCPP_INFO(
                    logger_, "[DartManager] cancel_launch completed, fire_count reset to 0");
            }

            current_task_.reset();
            transition_to(State::IDLE);

        } else if (status == ActionStatus::FAILURE) {
            RCLCPP_ERROR(
                logger_, "[DartManager] task '%s' FAILURE → state=ERROR",
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
        *force_control_velocity_ = 0.0; // 停止丝杆电机（已包含）
        clear_auto_aim_feedback();

        transition_to(State::ERROR);
    }

    void enter_belt_wait_zero_velocity_mode() {
        *belt_command_ = rmcs_msgs::DartSliderStatus::WAIT;
        *belt_target_velocity_ = 0.0;
        *belt_hold_torque_ = 0.0;
        *belt_torque_offset_ = 0.0;
        *belt_wait_zero_velocity_ = true;
        *belt_error_gain_ = 1.0;
        *belt_use_decel_pid_ = false;
    }

    void transition_to(State new_state) {
        state_ = new_state;
        first_tick_of_task_ = true;

        if (state_pub_) {
            std_msgs::msg::UInt8 msg;
            msg.data = static_cast<uint8_t>(new_state);
            state_pub_->publish(msg);
        }
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
            const auto& values = parameter.as_integer_array();
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
        aim_accept_deadband_px_ = has_parameter("aim_accept_deadband_px")
                                    ? get_vector2_parameter_or_throw("aim_accept_deadband_px")
                                    : aim_ready_exit_deadband_px_;
        aim_yaw_gain_ = get_numeric_parameter_or_throw("aim_yaw_gain");
        aim_pitch_gain_ = get_numeric_parameter_or_throw("aim_pitch_gain");
        aim_ready_confirm_ticks_ = has_parameter("aim_ready_confirm_ticks")
                                     ? get_uint_parameter_or_throw("aim_ready_confirm_ticks")
                                     : 5;
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
        *aim_current_dart_index_ = launch_prepare_enable_visual_assist_
                                     ? dart_launch_sequence_.current_dart_index_u8()
                                     : static_cast<uint8_t>(0);
        auto_aim_feedback_.set_desired_target_px(current_desired_target_px());
    }

    void clear_auto_aim_feedback() {
        *aim_current_dart_index_ = launch_prepare_enable_visual_assist_
                                     ? dart_launch_sequence_.current_dart_index_u8()
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
            RCLCPP_WARN(logger_, "[DartManager] current dart index already at the last dart");
        }
        sync_current_dart_outputs();
        clear_auto_aim_feedback();
    }

    void prepare_outputs_for_task(const Task& task) {
        if (task.name() == "launch_preparation" || task.name() == "fire"
            || task.name() == "cancel_launch" || task.name() == "slider_init"
            || task.name() == "manual_angle" || task.name() == "manual_force") {
            clear_auto_aim_feedback();
        }
    }

    std::shared_ptr<Task> make_slider_init_task() {
        // 为视觉接口提供默认值，即使视觉模块未启动也能正常工作
        static Eigen::Vector2d default_target_position = Eigen::Vector2d::Zero();

        // yaw_pitch_control_angle_ 是 OutputInterface，总是有效的
        // target_positionfire_count_input_ 是 InputInterface，需要检查是否 ready
        const Eigen::Vector2d* target_position_ptr =
            target_position_input_.ready() ? &(*target_position_input_) : &default_target_position;

        RCLCPP_INFO(logger_, "slider_init_task_GO");
        return std::make_shared<SliderInitTask>(
            *belt_command_, *belt_target_velocity_, *belt_torque_limit_, *belt_hold_torque_,
            *belt_wait_zero_velocity_, *left_belt_velocity_, *right_belt_velocity_,
            *left_belt_torque_, *right_belt_torque_, *belt_zero_calibration_,
            &(*yaw_pitch_control_angle_), target_position_ptr, yaw_transform_rate_);
    }

    // 任务工厂
    std::shared_ptr<Task> make_task(const std::string& cmd) {
        if (cmd == "launch_prepare" || cmd == "launch-prepare") {
            // 根据当前 fire_count 选择下降速度（fire_count=0 表示第一次准备）
            double down_velocity = (fire_count_ == 0) ? 12.0 : 10.0;
            bool require_lifting_down = (fire_count_ > 0);
            bool is_first_shot = (fire_count_ == 0);

            // 打印角度反馈状态
            RCLCPP_INFO(
                logger_,
                "[DartManager] Creating launch_prepare task, fire_count=%u, is_first_shot=%d",
                fire_count_, is_first_shot);
            if (left_belt_angle_.ready() && right_belt_angle_.ready()) {
                RCLCPP_INFO(
                    logger_, "[DartManager] Belt angles: left=%.4f, right=%.4f", *left_belt_angle_,
                    *right_belt_angle_);
            } else {
                RCLCPP_WARN(logger_, "[DartManager] Belt angle feedback NOT READY!");
            }
            RCLCPP_INFO(
                logger_, "[DartManager] Belt params: down_distance=%.4f m, pulley_radius=%.4f m",
                belt_down_distance_, down_velocity);

            // 读取力值用于力矩闭环（不在此处记录）
            bool ch1_ready = current_force_ch1_.ready();
            bool ch2_ready = current_force_ch2_.ready();

            int force_ch1_value = ch1_ready ? *current_force_ch1_ : 0;
            int force_ch2_value = ch2_ready ? *current_force_ch2_ : 0;

            // 根据 force_channel_ 选择主控通道
            if (force_channel_ == 2 && ch2_ready) {
                last_fire_force_ = static_cast<double>(force_ch2_value);
            } else if (force_channel_ == 1 && ch1_ready) {
                last_fire_force_ = static_cast<double>(force_ch1_value);
            }

            // Prepare Kalman filter pointers (if enabled and ready)
            const double* kalman_force_ptr = nullptr;
            const double* kalman_rate_ptr = nullptr;
            if (use_kalman_force_ && kalman_filtered_force_.ready() && kalman_force_rate_.ready()) {
                kalman_force_ptr = &(*kalman_filtered_force_);
                kalman_rate_ptr = &(*kalman_force_rate_);
                RCLCPP_INFO(
                    logger_, "[DartManager] Using Kalman force: F_filt=%.1fN, dF/dt=%.1fN/s",
                    *kalman_force_ptr, *kalman_rate_ptr);
            } else if (use_kalman_force_) {
                RCLCPP_WARN(
                    logger_,
                    "[DartManager] use_kalman_force=true but Kalman inputs not ready, falling back "
                    "to raw sensors");
            }

            return std::make_shared<LaunchPreparationTask>(
                *belt_command_, *belt_target_velocity_, *belt_torque_limit_, *belt_hold_torque_,
                *belt_wait_zero_velocity_, *belt_torque_offset_, *left_belt_angle_,
                *right_belt_angle_, *left_belt_velocity_, *right_belt_velocity_,
                *trigger_lock_enable_, belt_down_distance_, down_velocity, require_lifting_down,
                *lifting_command_, *lifting_left_vel_fb_, *lifting_right_vel_fb_,
                *belt_zero_calibration_, *force_control_velocity_, *current_force_ch1_,
                *current_force_ch2_, force_channel_, last_fire_force_, enable_force_calibration_,
                force_tolerance_, force_settle_ticks_, force_timeout_ticks_, force_kp_, force_ki_,
                force_kd_, is_first_shot, use_kalman_force_, kalman_force_ptr, kalman_rate_ptr,
                kalman_rate_feedforward_, kalman_rate_gain_);
        }

        if (cmd == "unload" || cmd == "cancel_launch") {
            bool require_lifting_up = (fire_count_ > 0);
            return std::make_shared<CancelLaunchTask>(
                *belt_command_, *belt_target_velocity_, *belt_torque_limit_, *belt_hold_torque_,
                *belt_wait_zero_velocity_, *belt_torque_offset_, *left_belt_angle_,
                *right_belt_angle_, *left_belt_velocity_, *right_belt_velocity_,
                *trigger_lock_enable_, *lifting_command_, *lifting_left_vel_fb_,
                *lifting_right_vel_fb_, belt_down_distance_, *belt_zero_calibration_,
                *force_control_velocity_, require_lifting_up);
        }

        if (cmd == "fire") {
            // 触发力传感器记录（由独立的ForceSensorRecorder组件处理）
            *fire_trigger_ = true;
            RCLCPP_INFO(logger_, "[DartManager] Fire command received, trigger signal sent");

            bool is_first_shot = (fire_count_ == 0);
            return std::make_shared<FireAndPreloadTask>(
                *trigger_lock_enable_, *lifting_command_, *lifting_left_vel_fb_,
                *lifting_right_vel_fb_, *limiting_command_, is_first_shot);
        }

        if (cmd == "manual_angle") {
            auto task = std::make_shared<Task>("manual_angle", "手动 yaw/pitch 调整");
            task->then(
                std::make_shared<DartManualAngleControlAction>(
                    auto_aim_feedback_.yaw_pitch_control_velocity()[0],
                    auto_aim_feedback_.yaw_pitch_control_velocity()[1], *joystick_left_,
                    *joystick_right_, max_transform_rate_));
            return task;
        }

        if (cmd == "manual_force") {
            const double* force_screw_velocity_feedback =
                force_screw_velocity_fb_.ready() ? &*force_screw_velocity_fb_ : nullptr;
            const double* force_screw_torque_feedback =
                force_screw_torque_fb_.ready() ? &*force_screw_torque_fb_ : nullptr;

            if (force_screw_velocity_feedback == nullptr
                || force_screw_torque_feedback == nullptr) {
                RCLCPP_WARN(
                    logger_, "[DartManager] force screw feedback not ready, manual_force stall "
                             "detection disabled");
            }

            auto task = std::make_shared<Task>("manual_force", "手动力丝杆速度调整");
            task->then(
                std::make_shared<DartManualForceControlAction>(
                    *force_control_velocity_, *joystick_right_, max_transform_rate_,
                    manual_force_scale_, force_screw_velocity_feedback,
                    force_screw_torque_feedback));
            return task;
        }
        return nullptr;
    }

    rclcpp::Logger logger_;

    InputInterface<double> left_belt_velocity_;
    InputInterface<double> right_belt_velocity_;
    InputInterface<double> left_belt_torque_;
    InputInterface<double> right_belt_torque_;
    InputInterface<double> left_belt_angle_;
    InputInterface<double> right_belt_angle_;

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> target_position_input_;
    // InputInterface<bool> target_tracking_input_;

    // 升降速度反馈（FillingLiftAction 堵转检测用）
    InputInterface<double> lifting_left_vel_fb_;
    InputInterface<double> lifting_right_vel_fb_;
    InputInterface<double> force_screw_velocity_fb_;
    InputInterface<double> force_screw_torque_fb_;

    // 力传感器反馈（两个通道）
    InputInterface<int> current_force_ch1_;
    InputInterface<int> current_force_ch2_;

    // Kalman filter inputs (optional)
    InputInterface<double> kalman_filtered_force_;
    InputInterface<double> kalman_force_rate_;

    OutputInterface<rmcs_msgs::DartSliderStatus> belt_command_;
    OutputInterface<double> belt_target_velocity_;
    OutputInterface<double> belt_torque_limit_;
    OutputInterface<double> belt_hold_torque_;
    OutputInterface<double> belt_torque_offset_;
    OutputInterface<bool> belt_wait_zero_velocity_;
    OutputInterface<bool> belt_zero_calibration_;
    OutputInterface<double> belt_error_gain_;
    OutputInterface<bool> belt_use_decel_pid_;
    OutputInterface<bool> trigger_lock_enable_;
    OutputInterface<bool> fire_trigger_; // 力传感器记录触发信号

    OutputInterface<Eigen::Vector2d> yaw_pitch_control_velocity_;
    OutputInterface<Eigen::Vector2d> yaw_pitch_control_angle_;
    OutputInterface<double> force_control_velocity_;
    OutputInterface<bool> aim_ready_;
    OutputInterface<uint8_t> aim_current_dart_index_;
    OutputInterface<Eigen::Vector2d> aim_error_px_;
    OutputInterface<Eigen::Vector2d> aim_desired_target_px_;

    OutputInterface<rmcs_msgs::DartSliderStatus> lifting_command_;
    OutputInterface<rmcs_msgs::DartLimitingServoStatus> limiting_command_;

    double max_transform_rate_{500.0};
    double yaw_transform_rate_;
    double manual_force_scale_{5.0};
    double auto_aim_max_transform_rate_{500.0};

    double belt_down_distance_{0.0};     // m

    // 像素到角度映射参数
    bool enable_pixel_to_angle_{false};

    // 力矩闭环参数
    bool enable_force_calibration_{false};
    double force_tolerance_{5.0}; // N
    uint64_t force_settle_ticks_{50};
    uint64_t force_timeout_ticks_{2000};
    double force_kp_{0.1};
    double force_ki_{0.0};
    double force_kd_{0.01};
    int force_channel_{1};        // 1 = ch1, 2 = ch2

    // Kalman filter force calibration
    bool use_kalman_force_{false};
    bool kalman_rate_feedforward_{false};
    double kalman_rate_gain_{0.0};

    bool launch_prepare_enable_visual_assist_{false};
    AutoAimFeedback auto_aim_feedback_;
    DartLaunchSequence dart_launch_sequence_;
    Eigen::Vector2d aim_deadband_px_{Eigen::Vector2d::Constant(3.0)};
    Eigen::Vector2d aim_ready_exit_deadband_px_{Eigen::Vector2d::Constant(5.0)};
    Eigen::Vector2d aim_accept_deadband_px_{Eigen::Vector2d::Constant(8.0)};
    double aim_yaw_gain_{0.0};
    double aim_pitch_gain_{0.0};
    uint64_t aim_ready_confirm_ticks_{5};
    double aim_min_transform_rate_{0.0};
    InputInterface<std::string> remote_command_input_;
    InputInterface<std::string> web_command_input_;
    std::string last_command_;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;

    State state_{State::IDLE};

    std::shared_ptr<Task> current_task_;
    std::deque<std::shared_ptr<Task>> task_queue_;
    bool first_fill_pending_{true};
    uint32_t fire_count_{0};      // 当前轮次已完成发射数
    bool first_tick_of_task_{true};
    double last_fire_force_{0.0}; // 上次fire前记录的力值

    bool temporary_flag_ = true;

    InputInterface<Eigen::Vector2d> target_position_;
};

} // namespace rmcs_dart_guidance::manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::manager::DartManager, rmcs_executor::Component)
