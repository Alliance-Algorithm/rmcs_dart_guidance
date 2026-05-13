#pragma once

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/belt_control_action.hpp"
#include <memory>

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// BeltInitTask
//   同步带初始化任务：通过一次低速上行堵转检测，将同步带推到上方机械基准位。
//   任务只包含一个 BeltControlAction，成功条件为稳定检测到堵转。
// ─────────────────────────────────────────────────────────────────────────────
class BeltInitTask : public Task {
public:
    BeltInitTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings)
        : Task("belt_init", "传送带上行复位") {

        then(
            std::make_shared<BeltControlAction>(
                "belt_up",                                   // 动作名称
                output.belt_command,                         // 同步带命令接口
                output.belt_target_velocity,                 // 同步带目标速度接口
                output.belt_exit_mode,                       // 电机退出状态接口
                output.belt_max_torque_override,             // 电机力矩上限覆盖接口
                input.belt_left_velocity,                    // 左电机速度反馈
                input.belt_left_torque,                      // 左电机力矩反馈
                input.belt_right_velocity,                   // 右电机速度反馈
                input.belt_right_torque,                     // 右电机力矩反馈
                rmcs_msgs::DartMechanismCommand::UP,         // 同步带命令设置
                settings.belt_init_setting_velocity,         // 同步带目标速度设置
                rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY,     // 电机退出模式设置
                settings.belt_init_stall_velocity_threshold,// 堵转速度阈值
                settings.belt_init_stall_torque_threshold,   // 堵转力矩阈值
                settings.belt_init_stall_confirm_ticks,      // 堵转确认帧数
                10000,                                       // 超时时间 ms
                settings.belt_init_max_torque                // 力矩上限
                ));
    }
};

} // namespace rmcs_dart_guidance::manager
