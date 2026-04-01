#include <string>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// RemoteCommandBridge
//   将遥控器 DR16 的拨杆/摇杆信号翻译为 DartManager 可识别的命令字和控制量。
//
//   键位映射：
//     ┌──────────────────┬──────────────────────────────────────────────────────┐
//     │ 左拨杆  右拨杆   │ 功能                                               │
//     ├──────────────────┼──────────────────────────────────────────────────────┤
//     │ DOWN    DOWN     │ 全部停止 → "cancel"                                │
//     │ MIDDLE  MIDDLE   │ 初始状态 → "recover"                               │
//     │ MIDDLE  DOWN→MID │ 切换上膛/退膛 → "launch_prepare" / "unload"        │
//     │ MIDDLE  UP       │ 处于上膛状态时发射 → "fire"                        │
//     │ UP      MIDDLE   │ 设置模式：摇杆调整 yaw/pitch                       │
//     │ UP      DOWN     │ 设置模式：调整拉力                                 │
//     └──────────────────┴──────────────────────────────────────────────────────┘
//
//   所有信息通过 Interface 传递，不持有 DartManager 指针。
// ─────────────────────────────────────────────────────────────────────────────
class RemoteCommandBridge
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    RemoteCommandBridge()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        // ── 遥控器输入 ───────────────────────────────────────────────────────
        register_input("/remote/switch/left", switch_left_, false);
        register_input("/remote/switch/right", switch_right_, false);
        register_input("/remote/joystick/left", joystick_left_, false);
        register_input("/remote/joystick/right", joystick_right_, false);

        // ── 命令输出（写给 DartManager 读取）─────────────────────────────────
        register_output("/dart/manager/command", command_output_, std::string{});

        RCLCPP_INFO(logger_, "[RemoteCommandBridge] initialized");
    }

    void update() override {
        using namespace rmcs_msgs;
        // using Switch = rmcs_msgs::Switch;

        auto left = *switch_left_;
        auto right = *switch_right_;

        // ── 最高优先级：双下 → 全部停止 ──────────────────────────────────────
        if (left == Switch::DOWN && right == Switch::DOWN) {
            emit_command("cancel");
            // RCLCPP_INFO(logger_,"cmd ： cancel");

            chambered_ = false;
            current_manual_mode_ = "";
            prev_right_ = right;
            return;
        }

        // ── 双中 → 初始状态（recover）────────────────────────────────────────
        if (left == Switch::MIDDLE && right == Switch::MIDDLE) {
            emit_command("recover");
            // RCLCPP_INFO(logger_,"cmd ： recover");
            current_manual_mode_ = "";
            detect_toggle(right); // 更新边沿状态但不执行动作
            return;
        }

        // ── 左中：操控模式 ───────────────────────────────────────────────────
        if (left == Switch::MIDDLE) {
            // 右拨杆 DOWN→MIDDLE 边沿：切换上膛/退膛
            if (detect_toggle(right)) {
                if (chambered_) {
                    emit_command("unload");
                    chambered_ = false;
                    RCLCPP_INFO(logger_, "[RemoteCommandBridge] toggle -> unload");
                } else {
                    emit_command("launch_prepare");
                    chambered_ = true;
                    RCLCPP_INFO(logger_, "[RemoteCommandBridge] toggle -> launch_prepare");
                }
                return;
            }

            // 处于上膛状态时，右拨杆上 → 发射
            if (chambered_ && right == Switch::UP) {
                emit_command("fire");
                chambered_ = false; // 发射后自动退出上膛状态
                RCLCPP_INFO(logger_, "[RemoteCommandBridge] fire!");
                return;
            }

            // 非触发帧：清空命令（允许 DartManager 重置去重状态）
            emit_command("");
            return;
        }

        // ── 左上：设置模式 ───────────────────────────────────────────────────
        if (left == Switch::UP) {
            std::string target_mode;
            if (right == Switch::MIDDLE) {
                target_mode = "manual_angle";
            } else if (right == Switch::DOWN) {
                target_mode = "manual_force";
            }

            if (current_manual_mode_ != target_mode) {
                // 需要切换模式或退出某手动模式，先发 cancel 打断现有任务
                emit_command("cancel");
                current_manual_mode_ = target_mode;
            } else {
                // 维持当前模式
                if (target_mode.empty()) {
                    emit_command("");
                } else {
                    emit_command(target_mode);
                }
            }
            prev_right_ = right;
            return;
        }

        // 离开设置模式时：
        if (!current_manual_mode_.empty()) {
            emit_command("cancel");
            current_manual_mode_ = "";
            prev_right_ = right;
            return;
        }

        // 未覆盖的组合（如 left==DOWN right!=DOWN）：安全清空
        emit_command("");
        prev_right_ = right;
    }

private:
    // ── 发送命令（写入 OutputInterface）───────────────────────────────────────
    void emit_command(const std::string& cmd) { *command_output_ = cmd; }

    // ── 边沿检测：右拨杆 DOWN→MIDDLE 触发一次 ────────────────────────────────
    //   返回 true 表示检测到一次 toggle 边沿
    bool detect_toggle(rmcs_msgs::Switch current_right) {
        bool triggered =
            (prev_right_ == rmcs_msgs::Switch::MIDDLE && current_right == rmcs_msgs::Switch::DOWN);
        prev_right_ = current_right;
        return triggered;
    }

    // ── 成员变量 ──────────────────────────────────────────────────────────────
    rclcpp::Logger logger_;

    // 遥控器输入
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<Eigen::Vector2d> joystick_right_;

    // 命令输出
    OutputInterface<std::string> command_output_;

    // 内部状态
    rmcs_msgs::Switch prev_right_{rmcs_msgs::Switch::UNKNOWN};
    bool chambered_{false};           // 是否处于上膛状态
    std::string current_manual_mode_; // 当前激活的手动控制模式
};

} // namespace rmcs_dart_guidance::manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::manager::RemoteCommandBridge, rmcs_executor::Component)
