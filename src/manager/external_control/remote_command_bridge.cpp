#include <string>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_dart_guidance::manager {

// RemoteCommandBridge
//   将遥控器 DR16 的拨杆信号翻译为 DartManager 可识别的离散命令。

/* 键位映射：
    双下：全部停止 -> "cancel"
    双上：预留自定义命令 -> 参数 "double_up_command"
    左拨杆 DOWN->MIDDLE：恢复 -> "recover"
    左拨杆在中：
        右拨杆 MIDDLE->DOWN：切换上膛/退膛 -> "launch_prepare" / "launch_cancel"
        右拨杆 MIDDLE->UP：处于上膛状态时发射 -> "fire_preload"
    左拨杆进入 UP：发一次手动控制 -> "manual_control"
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

        register_output("/dart/manager/command", command_output_, std::string{});

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
    }

    void update() override {
        using namespace rmcs_msgs;

        emit_command("");

        const auto left = *switch_left_;
        const auto right = *switch_right_;

        if (left == Switch::DOWN && right == Switch::DOWN) {
            emit_command("cancel");
            chambered_ = false;
            // RCLCPP_INFO(logger_, "[RemoteCommandBridge] cancel");
            update_previous_switches(left, right);
            return;
        }

        if (detect_carriage_init_transition(left, right)) {
            emit_command("carriage_init");
            RCLCPP_INFO(logger_, "[RemoteCommandBridge] carriage_init");
            update_previous_switches(left, right);
            return;
        }

        if (detect_enter_manual_control(left)) {
            emit_command("manual_control");
            RCLCPP_INFO(logger_, "[RemoteCommandBridge] enter manual_control");
            update_previous_switches(left, right);
            return;
        }

        if (detect_recover_transition(left)) {
            emit_command("recover");
            chambered_ = false;
            RCLCPP_INFO(logger_, "[RemoteCommandBridge] recover");
            update_previous_switches(left, right);
            return;
        }

        if (left == Switch::MIDDLE) {
            if (detect_prepare_toggle(left, right)) {
                if (chambered_) {
                    emit_command("launch_cancel");
                    chambered_ = false;
                    RCLCPP_INFO(logger_, "[RemoteCommandBridge] prepare toggle -> launch_cancel");
                } else {
                    emit_command("launch_prepare");
                    // emit_command("launch_prepare_with_vision");
                    chambered_ = true;
                    RCLCPP_INFO(logger_, "[RemoteCommandBridge] prepare toggle -> launch_prepare");
                }
                update_previous_switches(left, right);
                return;
            }

            if (chambered_ && detect_fire_transition(left, right)) {
                emit_command("fire_preload");
                chambered_ = false;
                RCLCPP_INFO(logger_, "[RemoteCommandBridge] fire_preload");
                update_previous_switches(left, right);
                return;
            }
        }

        update_previous_switches(left, right);
    }

private:
    void emit_command(const std::string& cmd) { *command_output_ = cmd; }

    bool detect_carriage_init_transition(
        rmcs_msgs::Switch current_left, rmcs_msgs::Switch current_right) const {
        return current_left == rmcs_msgs::Switch::UP && current_right == rmcs_msgs::Switch::DOWN
            && (prev_left_ != rmcs_msgs::Switch::UP || prev_right_ != rmcs_msgs::Switch::DOWN);
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

    void update_previous_switches(rmcs_msgs::Switch current_left, rmcs_msgs::Switch current_right) {
        prev_left_ = current_left;
        prev_right_ = current_right;
    }

    rclcpp::Logger logger_;

    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    OutputInterface<std::string> command_output_;

    rmcs_msgs::Switch prev_left_{rmcs_msgs::Switch::UNKNOWN};
    rmcs_msgs::Switch prev_right_{rmcs_msgs::Switch::UNKNOWN};
    bool chambered_{false};
};

} // namespace rmcs_dart_guidance::manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::manager::RemoteCommandBridge, rmcs_executor::Component)
