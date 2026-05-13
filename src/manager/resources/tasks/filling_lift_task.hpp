#pragma once

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/filling_lift_action.hpp"

#include <memory>

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// FillingLiftUpTask
//   填装机构上行任务：执行一次向上的 FillingLiftAction，直到堵转或超时结束。
// ─────────────────────────────────────────────────────────────────────────────
class FillingLiftUpTask : public Task {
public:
    FillingLiftUpTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings)
        : Task("filling_lift_up", "填装机构上行") {

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
                20000));                                 // 超时 tick
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// FillingLiftDownTask
//   填装机构下行任务：执行一次向下的 FillingLiftAction，直到堵转或超时结束。
// ─────────────────────────────────────────────────────────────────────────────
class FillingLiftDownTask : public Task {
public:
    FillingLiftDownTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings)
        : Task("filling_lift_down", "填装机构下行") {

        then(
            std::make_shared<FillingLiftAction>(
                "filling_lift_down",                     // 动作名称
                output.lifting_command,                  // 升降命令接口
                output.lift_target_velocity,             // 升降目标速度接口
                output.lift_exit_mode,                   // 升降退出模式接口
                input.lift_left_velocity,                // 左侧升降速度反馈
                input.lift_left_torque,                  // 左侧升降力矩反馈
                input.lift_right_velocity,               // 右侧升降速度反馈
                input.lift_right_torque,                 // 右侧升降力矩反馈
                rmcs_msgs::DartMechanismCommand::DOWN,   // 升降方向
                settings.lift_target_velocity,           // 升降目标速度
                rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY, // 退出模式
                settings.lift_stall_velocity_threshold,  // 堵转速度阈值
                settings.lift_stall_torque_threshold,    // 堵转力矩阈值
                settings.lift_stall_confirm_ticks,       // 堵转确认帧数
                20000));                                 // 超时 tick
    }
};

} // namespace rmcs_dart_guidance::manager
