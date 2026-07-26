#include <string>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_dart_guidance::manager {

// RemoteCommandBridge
//   将遥控器 DR16 输入翻译为 DartManager 可识别的离散命令。

/* 键位映射：
    双下：全部停止 -> "cancel"
    左拨杆 DOWN->MIDDLE：恢复 -> "recover"
    左拨杆保持 MIDDLE，右拨杆 MIDDLE->DOWN：发射准备 -> "dart-launch-prepare"
    左拨杆保持 MIDDLE，右拨杆 MIDDLE->UP：发射 -> "dart-fire"

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
            remember_switches(left, right);
            return;
        }

        if (detect_recover_transition(left)) {
            emit_command("recover");
            RCLCPP_INFO(logger_, "[RemoteCommandBridge] recover");
            remember_switches(left, right);
            return;
        }

        if (detect_launch_prepare_transition(left, right)) {
            emit_command("dart-launch-prepare");
            RCLCPP_INFO(logger_, "[RemoteCommandBridge] dart-launch-prepare");
            remember_switches(left, right);
            return;
        }

        if (detect_fire_transition(left, right)) {
            emit_command("dart-fire");
            RCLCPP_INFO(logger_, "[RemoteCommandBridge] dart-fire");
            remember_switches(left, right);
            return;
        }

        remember_switches(left, right);
    }

private:
    void emit_command(const std::string& cmd) { *command_output_ = cmd; }

    void remember_switches(rmcs_msgs::Switch left, rmcs_msgs::Switch right) {
        prev_left_ = left;
        prev_right_ = right;
    }

    bool detect_recover_transition(rmcs_msgs::Switch current_left) const {
        return current_left == rmcs_msgs::Switch::MIDDLE && prev_left_ == rmcs_msgs::Switch::DOWN;
    }

    bool detect_launch_prepare_transition(
        rmcs_msgs::Switch current_left, rmcs_msgs::Switch current_right) const {
        return current_left == rmcs_msgs::Switch::MIDDLE
            && prev_right_ == rmcs_msgs::Switch::MIDDLE
            && current_right == rmcs_msgs::Switch::DOWN;
    }

    bool detect_fire_transition(
        rmcs_msgs::Switch current_left, rmcs_msgs::Switch current_right) const {
        return current_left == rmcs_msgs::Switch::MIDDLE && prev_right_ == rmcs_msgs::Switch::MIDDLE
            && current_right == rmcs_msgs::Switch::UP;
    }

    rclcpp::Logger logger_;

    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    OutputInterface<std::string> command_output_;

    rmcs_msgs::Switch prev_left_{rmcs_msgs::Switch::UNKNOWN};
    rmcs_msgs::Switch prev_right_{rmcs_msgs::Switch::UNKNOWN};
};

} // namespace rmcs_dart_guidance::manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::manager::RemoteCommandBridge, rmcs_executor::Component)
