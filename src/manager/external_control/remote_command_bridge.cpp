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
//   将遥控器 DR16 和外部 ROS topic 输入翻译为 DartManager 可识别的离散命令。

/* 键位映射：
    双下：全部停止 -> "cancel"
    左拨杆 DOWN->MIDDLE：恢复 -> "recover"
    左拨杆在中：
        右拨杆 MIDDLE->DOWN：切换上膛/退膛 -> "launch_prepare" / "launch_cancel"
        右拨杆 MIDDLE->UP：处于上膛状态时发射 -> "fire_preload"
    左拨杆进入 UP：发一次手动控制 -> "manual_control"

    外部 ROS：
        /carriage_position/calibrate 非 0：发一次 "carriage_init"
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
        register_input("/referee/dart/launch_remain_time", dart_remaining_time_, false);

        register_output("/dart/manager/command", command_output_, std::string{});

        carriage_position_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/carriage_position/calibrate", rclcpp::QoS{10},
            [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                carriage_position_calibrate_subscription_callback(std::move(msg));
            });

        vision_enable_ = get_parameter("vision_enable").as_bool();
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
            RCLCPP_WARN(
                logger_, "Failed to fetch \"/referee/dart/launch_remain_time\". Set to 0.");
        }
    }

    void update() override {
        using namespace rmcs_msgs;

        emit_command("");

        if (carriage_init_receive_) {
            emit_command("carriage_init");
            carriage_init_receive_ = false;
        }

        const auto left = *switch_left_;
        const auto right = *switch_right_;
        const auto knob = *rotary_knob_switch_;
        const auto game_stage = *game_stage_;
        const auto dart_remaining_time = *dart_remaining_time_;

        if (const auto command = detect_station_open_command(
                left, right, knob, game_stage, dart_remaining_time);
            !command.empty()) {
            emit_command(command);
            chambered_ = false;
            update_previous_inputs(left, right, knob, dart_remaining_time);
            RCLCPP_INFO(logger_, "[RemoteCommandBridge] station open trigger -> %s", command.c_str());
            return;
        }

        if (left == Switch::DOWN && right == Switch::DOWN) {
            emit_command("cancel");
            chambered_ = false;
            update_previous_inputs(left, right, knob, dart_remaining_time);
            return;
        }

        if (detect_enter_manual_control(left)) {
            emit_command("manual_control");
            RCLCPP_INFO(logger_, "[RemoteCommandBridge] enter manual_control");
            update_previous_inputs(left, right, knob, dart_remaining_time);
            return;
        }

        if (detect_recover_transition(left)) {
            emit_command("recover");
            chambered_ = false;
            RCLCPP_INFO(logger_, "[RemoteCommandBridge] recover");
            update_previous_inputs(left, right, knob, dart_remaining_time);
            return;
        }

        if (left == Switch::MIDDLE) {
            if (detect_prepare_toggle(left, right)) {
                if (chambered_) {
                    emit_command("launch_cancel");
                    chambered_ = false;
                    RCLCPP_INFO(logger_, "[RemoteCommandBridge] prepare toggle -> launch_cancel");
                } else {
                    const char* fire_task_name =
                        vision_enable_ ? "launch_prepare_with_vision" : "launch_prepare";
                    emit_command(fire_task_name);
                    chambered_ = true;
                    RCLCPP_INFO(
                        logger_, "[RemoteCommandBridge] prepare toggle -> %s", fire_task_name);
                }
                update_previous_inputs(left, right, knob, dart_remaining_time);
                return;
            }

            if (chambered_ && detect_fire_transition(left, right)) {
                emit_command("fire_preload");
                chambered_ = false;
                RCLCPP_INFO(logger_, "[RemoteCommandBridge] fire_preload");
                update_previous_inputs(left, right, knob, dart_remaining_time);
                return;
            }
        }

        update_previous_inputs(left, right, knob, dart_remaining_time);
    }

private:
    void emit_command(const std::string& cmd) { *command_output_ = cmd; }

    void carriage_position_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr msg) {
        if (msg == nullptr || msg->data == 0) {
            return;
        }
        carriage_init_receive_ = true;
        // emit_command("carriage_init");
        RCLCPP_INFO(logger_, "[RemoteCommandBridge] /carriage_position/calibrate -> carriage_init");
    }

    bool detect_enter_manual_control(rmcs_msgs::Switch current_left) const {
        return current_left == rmcs_msgs::Switch::UP && prev_left_ != rmcs_msgs::Switch::UP;
    }

    bool detect_recover_transition(rmcs_msgs::Switch current_left) const {
        return current_left == rmcs_msgs::Switch::MIDDLE && prev_left_ == rmcs_msgs::Switch::DOWN;
    }

    bool detect_prepare_toggle(
        rmcs_msgs::Switch current_left, rmcs_msgs::Switch current_right) const {
        return current_left == rmcs_msgs::Switch::MIDDLE && prev_left_ == rmcs_msgs::Switch::MIDDLE
            && prev_right_ == rmcs_msgs::Switch::MIDDLE && current_right == rmcs_msgs::Switch::DOWN;
    }

    bool detect_fire_transition(
        rmcs_msgs::Switch current_left, rmcs_msgs::Switch current_right) const {
        return current_left == rmcs_msgs::Switch::MIDDLE && prev_left_ == rmcs_msgs::Switch::MIDDLE
            && prev_right_ == rmcs_msgs::Switch::MIDDLE && current_right == rmcs_msgs::Switch::UP;
    }

    std::string detect_station_open_command(
        rmcs_msgs::Switch current_left, rmcs_msgs::Switch current_right,
        rmcs_msgs::Switch current_knob, rmcs_msgs::GameStage current_game_stage,
        uint8_t current_dart_remaining_time) {
        if (current_game_stage == rmcs_msgs::GameStage::STARTED
            && prev_dart_remaining_time_ != 29 && current_dart_remaining_time == 29) {
            return next_station_open_command();
        }

        if (current_game_stage != rmcs_msgs::GameStage::STARTED
            && current_left == rmcs_msgs::Switch::DOWN
            && current_right == rmcs_msgs::Switch::UP
            && prev_knob_ != rmcs_msgs::Switch::UP
            && current_knob == rmcs_msgs::Switch::UP) {
            return next_station_open_command();
        }

        return {};
    }

    std::string next_station_open_command() {
        switch (station_open_trigger_count_) {
        case 0:
            ++station_open_trigger_count_;
            return "first_dart_station_open_task";
        case 1:
            ++station_open_trigger_count_;
            return "second_dart_station_open_task";
        default: return {};
        }
    }

    void update_previous_inputs(
        rmcs_msgs::Switch current_left, rmcs_msgs::Switch current_right,
        rmcs_msgs::Switch current_knob, uint8_t current_dart_remaining_time) {
        prev_left_ = current_left;
        prev_right_ = current_right;
        prev_knob_ = current_knob;
        prev_dart_remaining_time_ = current_dart_remaining_time;
    }

    rclcpp::Logger logger_;

    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> rotary_knob_switch_;
    InputInterface<rmcs_msgs::GameStage> game_stage_;
    InputInterface<uint8_t> dart_remaining_time_;
    OutputInterface<std::string> command_output_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr carriage_position_calibrate_subscription_;

    bool vision_enable_;
    std::atomic<bool> carriage_init_receive_;

    rmcs_msgs::Switch prev_left_{rmcs_msgs::Switch::UNKNOWN};
    rmcs_msgs::Switch prev_right_{rmcs_msgs::Switch::UNKNOWN};
    rmcs_msgs::Switch prev_knob_{rmcs_msgs::Switch::UNKNOWN};
    uint8_t prev_dart_remaining_time_{0};
    uint8_t station_open_trigger_count_{0};
    bool chambered_{false};
};

} // namespace rmcs_dart_guidance::manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::manager::RemoteCommandBridge, rmcs_executor::Component)
