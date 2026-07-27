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
//   将遥控器 DR16 输入翻译为 DartManager 可识别的离散命令。

/* 键位映射：
    双下：全部停止 -> "cancel"
    左拨杆 DOWN->MIDDLE：恢复 -> "recover"
    左拨杆保持 MIDDLE，右拨杆 MIDDLE->DOWN：发射准备/取消准备 toggle
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
    }

    void update() override {
        using namespace rmcs_msgs;

        emit_command("");

        const auto left = *switch_left_;
        const auto right = *switch_right_;

        if (left == Switch::DOWN && right == Switch::DOWN) {
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

    rclcpp::Logger logger_;

    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
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
};

} // namespace rmcs_dart_guidance::manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::manager::RemoteCommandBridge, rmcs_executor::Component)
