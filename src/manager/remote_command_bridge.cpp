#include <atomic>
#include <string>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <std_msgs/msg/int32.hpp>

namespace rmcs_dart_guidance::manager {

// RemoteCommandBridge
//   将遥控器 DR16 和外部 ROS topic 输入翻译为 DartManager 可识别的离散命令。

/* 键位映射（belt/trigger/filling 任务已停用，仅保留 cancel/recover/example）：
    双下：全部停止 -> "cancel"
    左拨杆 DOWN->MIDDLE：恢复 -> "recover"

    外部 ROS：
        /dart/example/trigger 非 0：发一次 "example"
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

        example_trigger_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/dart/example/trigger", rclcpp::QoS{10},
            [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                example_trigger_subscription_callback(std::move(msg));
            });

        RCLCPP_INFO(logger_, "[RemoteCommandBridge] initialized (example-only dispatch)");
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

        if (example_receive_) {
            emit_command("example");
            example_receive_ = false;
        }

        const auto left = *switch_left_;
        const auto right = *switch_right_;

        if (left == Switch::DOWN && right == Switch::DOWN) {
            emit_command("cancel");
            prev_left_ = left;
            return;
        }

        if (detect_recover_transition(left)) {
            emit_command("recover");
            RCLCPP_INFO(logger_, "[RemoteCommandBridge] recover");
            prev_left_ = left;
            return;
        }

        prev_left_ = left;
    }

private:
    void emit_command(const std::string& cmd) { *command_output_ = cmd; }

    void example_trigger_subscription_callback(std_msgs::msg::Int32::UniquePtr msg) {
        if (msg == nullptr || msg->data == 0) {
            return;
        }
        example_receive_ = true;
        RCLCPP_INFO(logger_, "[RemoteCommandBridge] /dart/example/trigger -> example");
    }

    bool detect_recover_transition(rmcs_msgs::Switch current_left) const {
        return current_left == rmcs_msgs::Switch::MIDDLE && prev_left_ == rmcs_msgs::Switch::DOWN;
    }

    rclcpp::Logger logger_;

    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    OutputInterface<std::string> command_output_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr example_trigger_subscription_;

    std::atomic<bool> example_receive_{false};

    rmcs_msgs::Switch prev_left_{rmcs_msgs::Switch::UNKNOWN};
};

} // namespace rmcs_dart_guidance::manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::manager::RemoteCommandBridge, rmcs_executor::Component)
