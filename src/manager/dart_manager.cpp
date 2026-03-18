#include "manager/action/action.hpp"
#include "manager/action/manual_angle_control.hpp"
#include "manager/action/manual_force_control.hpp"
#include "manager/task/cancel_launch_task.hpp"
#include "manager/task/fire_and_preload_task.hpp"
#include "manager/task/launch_preparation_task.hpp"
#include "manager/task/silder_init_task.hpp"
#include "manager/task/task.hpp"

#include <cstdint>
#include <deque>
#include <memory>
#include <string>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
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

        register_output("/dart/manager/belt/command", belt_command_, rmcs_msgs::DartSliderStatus::WAIT);
        register_output("/dart/manager/belt/target_velocity", belt_target_velocity_, 0.0);
        register_output("/dart/manager/belt/torque_limit", belt_torque_limit_, 0.0);
        register_output("/dart/manager/belt/hold_torque", belt_hold_torque_, 0.0);
        register_output("/dart/manager/belt/wait_zero_velocity", belt_wait_zero_velocity_, false);
        register_output("/dart/manager/trigger/lock_enable", trigger_lock_enable_, false);

        register_output("/pitch/control/velocity", yaw_pitch_control_velocity_, Eigen::Vector2d::Zero());
        register_output("/force/control/velocity", force_control_velocity_, 0.0);

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

        limiting_fill_ticks_  = (uint64_t)get_parameter("limiting_fill_ticks").as_int();

        lifting_stall_threshold_    = get_parameter("lifting_stall_threshold").as_double();
        lifting_stall_confirm_ticks_ =
            (uint64_t)get_parameter("lifting_stall_confirm_ticks").as_int();
        lifting_stall_min_run_ticks_ =
            (uint64_t)get_parameter("lifting_stall_min_run_ticks").as_int();
        lifting_stall_timeout_ticks_ =
            (uint64_t)get_parameter("lifting_stall_timeout_ticks").as_int();

        state_pub_ = create_publisher<std_msgs::msg::UInt8>("/dart/manager/state", 10);

        submit_task(make_slider_init_task());
        RCLCPP_INFO(logger_, "[DartManagerV2] initialized, queued SliderInitTask on startup");
    }

    void update() override {
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

        // 无论 ERROR 还是 IDLE，都重新排队传送带复位
        submit_task(make_slider_init_task());
        RCLCPP_INFO(logger_, "[DartManagerV2] queued SliderInitTask for recovery");
    }

    void dispatch_next_task() {
        if (task_queue_.empty())
            return;

        current_task_ = std::move(task_queue_.front());
        task_queue_.pop_front();

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
            current_task_->on_exit();
            RCLCPP_INFO(
                logger_, "[DartManagerV2] task '%s' SUCCESS", current_task_->name().c_str());
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

            return std::make_shared<LaunchPreparationTask>(
                *belt_command_, *belt_target_velocity_, *belt_torque_limit_, *belt_hold_torque_,
                *belt_wait_zero_velocity_,
                *left_belt_velocity_, *right_belt_velocity_,
                *left_belt_torque_,   *right_belt_torque_,
                *trigger_lock_enable_,
                *lifting_command_,
                *lifting_left_vel_fb_, *lifting_right_vel_fb_,
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
                (*yaw_pitch_control_velocity_)[0], (*yaw_pitch_control_velocity_)[1],
                *joystick_left_, *joystick_right_,
                max_transform_rate_));
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

    OutputInterface<rmcs_msgs::DartSliderStatus> lifting_command_;
    OutputInterface<rmcs_msgs::DartLimitingServoStatus> limiting_command_;

    double   max_transform_rate_{500.0};
    double   manual_force_scale_{5.0};
    uint64_t limiting_fill_ticks_{500};

    double   lifting_stall_threshold_{0.5};
    uint64_t lifting_stall_confirm_ticks_{100};
    uint64_t lifting_stall_min_run_ticks_{500};
    uint64_t lifting_stall_timeout_ticks_{5000};

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
