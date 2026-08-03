#include <atomic>
#include <chrono>
#include <optional>
#include <string>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/game_stage.hpp>
#include <rmcs_msgs/switch.hpp>
#include <std_msgs/msg/int32.hpp>

namespace rmcs_dart_guidance::manager {

// RemoteCommandBridge
//   将遥控器 DR16 输入翻译为 DartManager 可识别的离散命令，并负责比赛控制触发。

/* 键位映射：
     双下：全部停止 -> "cancel"
     左拨杆 DOWN->MIDDLE：恢复 -> "recover"
     左拨杆保持 MIDDLE，右拨杆 MIDDLE->DOWN：发射准备/取消准备 toggle
     左拨杆保持 MIDDLE，右拨杆 MIDDLE->UP：发射 -> "dart-fire"
     左拨杆 UP：手动模式，抑制所有手柄命令（比赛任务不受影响）
*/

class RemoteCommandBridge
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    RemoteCommandBridge()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        register_input("/remote/switch/left", switch_left_, false);
        register_input("/remote/switch/right", switch_right_, false);
        register_input("/remote/rotary_knob_switch", rotary_knob_switch_, false);
        register_input("/referee/game/stage", game_stage_, false);
        register_input("/referee/dart/remaining_time", dart_remaining_time_, false);

        register_output("/dart/manager/command", command_output_, std::string{});

        chassis_zero_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/chassis/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr) {
                chassis_zero_calibrate_pending_.store(true);
                RCLCPP_INFO(logger_, "[RemoteCommandBridge] chassis zero calibrate requested");
            });

        chassis_level_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/chassis/leveling", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr) {
                chassis_level_pending_.store(true);
                RCLCPP_INFO(logger_, "[RemoteCommandBridge] chassis level requested");
            });

        carriage_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/carriage/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr) {
                carriage_calibrate_pending_.store(true);
                RCLCPP_INFO(logger_, "[RemoteCommandBridge] carriage calibrate requested");
            });

        RCLCPP_INFO(logger_, "[RemoteCommandBridge] initialized");
    }

    void before_updating() override {
        if (!switch_left_.ready()) {
            switch_left_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(logger_, "Failed to fetch \"/remote/switch/left\". Set to UNKNOWN.");
        }
        if (!switch_right_.ready()) {
            switch_right_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(logger_, "Failed to fetch \"/remote/switch/right\". Set to UNKNOWN.");
        }
        if (!rotary_knob_switch_.ready()) {
            rotary_knob_switch_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(
                logger_, "Failed to fetch \"/remote/rotary_knob_switch\". Set to UNKNOWN.");
        }
        if (!game_stage_.ready()) {
            game_stage_.make_and_bind_directly(rmcs_msgs::GameStage::UNKNOWN);
            RCLCPP_WARN(logger_, "Failed to fetch \"/referee/game/stage\". Set to UNKNOWN.");
        }
        if (!dart_remaining_time_.ready()) {
            dart_remaining_time_.make_and_bind_directly(uint8_t{0});
            RCLCPP_WARN(logger_, "Failed to fetch \"/referee/dart/remaining_time\". Set to 0.");
        }
    }

    void update() override {
        using namespace rmcs_msgs;

        emit_command("");

        if (check_game_control_triggers()) {
            remember_switches(*switch_left_, *switch_right_);
            return;
        }

        const bool manual_mode = switch_left_.ready() && *switch_left_ == Switch::UP;
        if (manual_mode) {
            clear_external_pending_requests();
            remember_switches(*switch_left_, *switch_right_);
            return;
        }

        const auto left = *switch_left_;
        const auto right = *switch_right_;
        const bool game_started = *game_stage_ == GameStage::STARTED;

        if (!game_started && left == Switch::DOWN && right == Switch::DOWN) {
            emit_command("cancel");
            launch_prepare_pending_ = false;
            clear_external_pending_requests();
            remember_switches(left, right);
            return;
        }

        if (detect_recover_transition(left)) {
            emit_command("recover");
            launch_prepare_pending_ = false;
            clear_external_pending_requests();
            RCLCPP_INFO(logger_, "[RemoteCommandBridge] recover");
            remember_switches(left, right);
            return;
        }

        if (detect_launch_prepare_transition(left, right)) {
            if (launch_prepare_pending_) {
                emit_command("dart-launch-cancel");
                launch_prepare_pending_ = false;
                clear_external_pending_requests();
                RCLCPP_INFO(logger_, "[RemoteCommandBridge] dart-launch-cancel");
            } else {
                emit_command("dart-launch-prepare");
                launch_prepare_pending_ = true;
                clear_external_pending_requests();
                RCLCPP_INFO(logger_, "[RemoteCommandBridge] dart-launch-prepare");
            }
            remember_switches(left, right);
            return;
        }

        if (detect_fire_transition(left, right)) {
            emit_command("dart-fire");
            launch_prepare_pending_ = false;
            clear_external_pending_requests();
            RCLCPP_INFO(logger_, "[RemoteCommandBridge] dart-fire");
            remember_switches(left, right);
            return;
        }

        if (chassis_zero_calibrate_pending_.exchange(false)) {
            emit_command("dart-chassis-zero-calibrate");
            launch_prepare_pending_ = false;
            RCLCPP_INFO(logger_, "[RemoteCommandBridge] dart-chassis-zero-calibrate");
            remember_switches(left, right);
            return;
        }

        if (chassis_level_pending_.exchange(false)) {
            emit_command("dart-chassis-level");
            launch_prepare_pending_ = false;
            RCLCPP_INFO(logger_, "[RemoteCommandBridge] dart-chassis-level");
            remember_switches(left, right);
            return;
        }

        if (carriage_calibrate_pending_.exchange(false)) {
            emit_command("dart-carriage-calibrate");
            launch_prepare_pending_ = false;
            RCLCPP_INFO(logger_, "[RemoteCommandBridge] dart-carriage-calibrate");
            remember_switches(left, right);
            return;
        }

        remember_switches(left, right);
    }

private:
    void emit_command(const std::string& cmd) { *command_output_ = cmd; }

    void clear_external_pending_requests() {
        chassis_zero_calibrate_pending_.store(false);
        chassis_level_pending_.store(false);
        carriage_calibrate_pending_.store(false);
    }

    void remember_switches(rmcs_msgs::Switch left, rmcs_msgs::Switch right) {
        prev_left_ = left;
        prev_right_ = right;
    }

    bool detect_recover_transition(rmcs_msgs::Switch current_left) const {
        return current_left == rmcs_msgs::Switch::MIDDLE && prev_left_ == rmcs_msgs::Switch::DOWN;
    }

    bool detect_launch_prepare_transition(
        rmcs_msgs::Switch current_left, rmcs_msgs::Switch current_right) const {
        return current_left == rmcs_msgs::Switch::MIDDLE && prev_right_ == rmcs_msgs::Switch::MIDDLE
            && current_right == rmcs_msgs::Switch::DOWN;
    }

    bool detect_fire_transition(
        rmcs_msgs::Switch current_left, rmcs_msgs::Switch current_right) const {
        return current_left == rmcs_msgs::Switch::MIDDLE && prev_right_ == rmcs_msgs::Switch::MIDDLE
            && current_right == rmcs_msgs::Switch::UP;
    }

    bool check_game_control_triggers() {
        using namespace rmcs_msgs;

        const bool ref_condition = game_stage_.ready() && *game_stage_ == GameStage::STARTED
                                && dart_remaining_time_.ready() && *dart_remaining_time_ > 15;
        if (ref_condition && !referee_condition_was_true_) {
            referee_condition_was_true_ = true;
            clear_external_pending_requests();
            emit_command("dart-game-control");
            RCLCPP_INFO(logger_, "[RemoteCommandBridge] game control triggered by referee");
            return true;
        }
        referee_condition_was_true_ = ref_condition;

        const bool rotary_condition =
            switch_left_.ready() && switch_right_.ready() && rotary_knob_switch_.ready()
            && *switch_left_ == Switch::MIDDLE && *switch_right_ == Switch::MIDDLE
            && *rotary_knob_switch_ == Switch::UP;

        const auto now = std::chrono::steady_clock::now();
        if (!rotary_condition) {
            rotary_up_since_.reset();
            rotary_hold_consumed_ = false;
            return false;
        }

        if (!rotary_up_since_) {
            rotary_up_since_ = now;
            return false;
        }

        if (!rotary_hold_consumed_ && (now - *rotary_up_since_) >= kRotaryTriggerHoldDuration) {
            rotary_hold_consumed_ = true;
            clear_external_pending_requests();
            emit_command("dart-game-control");
            RCLCPP_INFO(logger_, "[RemoteCommandBridge] game control triggered by rotary");
            return true;
        }

        return false;
    }

    rclcpp::Logger logger_;

    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> rotary_knob_switch_;
    InputInterface<rmcs_msgs::GameStage> game_stage_;
    InputInterface<uint8_t> dart_remaining_time_;
    OutputInterface<std::string> command_output_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr chassis_zero_calibrate_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr chassis_level_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr carriage_calibrate_subscription_;

    rmcs_msgs::Switch prev_left_{rmcs_msgs::Switch::UNKNOWN};
    rmcs_msgs::Switch prev_right_{rmcs_msgs::Switch::UNKNOWN};
    bool launch_prepare_pending_{false};
    std::atomic_bool chassis_zero_calibrate_pending_{false};
    std::atomic_bool chassis_level_pending_{false};
    std::atomic_bool carriage_calibrate_pending_{false};

    bool referee_condition_was_true_{false};
    std::optional<std::chrono::steady_clock::time_point> rotary_up_since_;
    bool rotary_hold_consumed_{false};

    static constexpr auto kRotaryTriggerHoldDuration = std::chrono::seconds{3};
};

} // namespace rmcs_dart_guidance::manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::manager::RemoteCommandBridge, rmcs_executor::Component)
