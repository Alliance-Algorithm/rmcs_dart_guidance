#pragma once

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/delay_action.hpp"
#include "manager/resources/actions/filling_lift_action.hpp"
#include "manager/resources/actions/filling_limit_servo_action.hpp"
#include "manager/resources/actions/trigger_control_action.hpp"
#include "rmcs_msgs/dart_servo_command.hpp"

#include <memory>

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// FireAndPreloadTask
//   发射并预装填任务：先短暂延时后释放扳机完成一次发射；如果 fire_count > 0，
//   则继续执行填装机构上行和限位舵机释放/回锁，完成下一发的预装填准备。
// ─────────────────────────────────────────────────────────────────────────────
class FireAndPreloadTask : public Task {
public:
    FireAndPreloadTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings, const ManagerRuntimeState& runtime_state)
        : Task("fire_preload", "发射并预装填") {
        then(std::make_shared<DelayAction>(
            "fire_delay",     // 动作名称
            1000              // 发射前延时 tick
            ));
        then(
            std::make_shared<TriggerControlAction>(
                "trigger_free",                              // 动作名称
                output.trigger_command,                      // 扳机命令接口
                rmcs_msgs::DartServoCommand::FREE,           // 扳机释放命令
                2000                                         // 舵机稳定等待 tick
                ));

        if (runtime_state.fire_count > 0) {
            then(
                std::make_shared<FillingLiftAction>(
                    "filling_lift_up",                       // 动作名称
                    output.lifting_command,                  // 升降命令接口
                    output.lift_target_velocity,             // 升降目标速度接口
                    output.lift_exit_mode,                   // 升降退出模式接口
                    input.lift_left_velocity,                // 左侧升降速度反馈
                    input.lift_left_torque,                  // 左侧升降力矩反馈
                    input.lift_right_velocity,               // 右侧升降速度反馈
                    input.lift_right_torque,                 // 右侧升降力矩反馈
                    rmcs_msgs::DartMechanismCommand::UP,     // 升降方向
                    settings.lift_target_velocity,           // 升降目标速度
                    rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY, // 退出模式
                    settings.lift_stall_velocity_threshold,  // 堵转速度阈值
                    settings.lift_stall_torque_threshold,    // 堵转力矩阈值
                    settings.lift_stall_confirm_ticks,       // 堵转确认帧数
                    20000                                    // 超时 tick
                    ));

            then(
                std::make_shared<FillingLimitServoAction>(
                    "filling_limit_servo",                   // 动作名称
                    output.limiting_command,                 // 限位舵机状态（输出）
                    rmcs_msgs::DartServoCommand::FREE,       // 先释放
                    rmcs_msgs::DartServoCommand::LOCK,       // 再锁回
                    settings.limiting_fill_ticks             // 预装填持续帧数
                    ));
        }
    }
};

} // namespace rmcs_dart_guidance::manager
