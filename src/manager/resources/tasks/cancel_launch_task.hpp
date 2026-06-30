#pragma once

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/belt_control_action.hpp"
#include "manager/resources/actions/filling_lift_action.hpp"
#include "manager/resources/actions/trigger_control_action.hpp"

#include <limits>
#include <memory>
#include <vector>

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// CancelLaunchTask
//   取消发射任务：同步带上下行改为累计位移切段计划，task 只描述切换点序列；
//   保留扳机释放、末端堵转复位和升降回升逻辑。
// ─────────────────────────────────────────────────────────────────────────────
class CancelLaunchTask : public Task {
public:
    CancelLaunchTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings)
        : Task("launch_cancel", "取消发射") {
        const double no_torque_limit = std::numeric_limits<double>::quiet_NaN();

        then(
            std::make_shared<BeltDisplacementPlanAction>(
                "belt_down_plan",                    // 动作名称
                output.belt_command,                  // 同步带命令接口
                output.belt_target_velocity,          // 同步带目标速度接口
                output.belt_exit_mode,                // 电机退出状态接口
                output.belt_max_torque_override,      // 电机力矩上限覆盖接口
                input.belt_left_angle,                // 左电机角度反馈
                input.belt_right_angle,               // 右电机角度反馈
                settings.belt_down_setting_velocity,  // 同步带基础目标速度
                std::vector<BeltDisplacementSwitchPoint>{
                    {
                        settings.belt_down_travel_angle * 3.3,
                        1.8,
                        rmcs_msgs::DartMechanismCommand::DOWN,
                        rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY,
                        no_torque_limit,
                    },
                    {
                        settings.belt_down_travel_angle * 3.8,
                        1.0,
                        rmcs_msgs::DartMechanismCommand::DOWN,
                        rmcs_msgs::ExitMode::WAIT_HOLD_TORQUE,
                        no_torque_limit,
                    },
                },
                20000                                 // 超时时间 ms
                ));

        then(
            std::make_shared<TriggerControlAction>(
                "trigger_free",                             // 动作名称
                output.trigger_command,                     // 扳机命令接口
                rmcs_msgs::DartServoCommand::FREE,          // 扳机释放命令
                100                                         // 舵机稳定等待 tick
                ));

        then(
            std::make_shared<BeltDisplacementPlanAction>(
                "belt_up_plan",                      // 动作名称
                output.belt_command,                  // 同步带命令接口
                output.belt_target_velocity,          // 同步带目标速度接口
                output.belt_exit_mode,                // 电机退出状态接口
                output.belt_max_torque_override,      // 电机力矩上限覆盖接口
                input.belt_left_angle,                // 左电机角度反馈
                input.belt_right_angle,               // 右电机角度反馈
                settings.belt_up_setting_velocity,    // 同步带基础目标速度
                std::vector<BeltDisplacementSwitchPoint>{
                    {
                        settings.belt_up_travel_angle,
                        0.5,
                        rmcs_msgs::DartMechanismCommand::UP,
                        rmcs_msgs::ExitMode::KEEP,
                        no_torque_limit,
                    },
                    {
                        settings.belt_up_travel_angle * 2.3,
                        2.0,
                        rmcs_msgs::DartMechanismCommand::UP,
                        rmcs_msgs::ExitMode::KEEP,
                        no_torque_limit,
                    },
                },
                20000                               // 超时时间 ms
                ));

        then(
            std::make_shared<BeltControlAction>(
                "belt_up_stall",                           // 动作名称
                output.belt_command,                        // 同步带命令接口
                output.belt_target_velocity,                // 同步带目标速度接口
                output.belt_exit_mode,                      // 电机退出状态接口
                output.belt_max_torque_override,            // 电机力矩上限覆盖接口
                input.belt_left_velocity,                   // 左电机速度反馈
                input.belt_left_torque,                     // 左电机力矩反馈
                input.belt_right_velocity,                  // 右电机速度反馈
                input.belt_right_torque,                    // 右电机力矩反馈
                rmcs_msgs::DartMechanismCommand::UP,        // 同步带命令设置
                settings.belt_up_setting_velocity * 0.2,    // 同步带目标速度设置
                rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY,    // 电机退出模式设置
                settings.belt_stall_velocity_threshold,     // 堵转速度阈值
                settings.belt_stall_torque_threshold * 0.5, // 堵转力矩阈值
                settings.belt_stall_confirm_ticks,          // 堵转确认帧数
                20000                                       // 超时时间 ms
                ));
        then(
            std::make_shared<FillingLiftAction>(
                "filling_lift_up",                          // 动作名称
                output.lifting_command,                     // 升降命令接口
                output.lift_target_velocity,                // 升降目标速度接口
                output.lift_exit_mode,                      // 升降退出模式接口
                input.lift_left_velocity,                   // 左侧升降速度反馈
                input.lift_left_torque,                     // 左侧升降力矩反馈
                input.lift_right_velocity,                  // 右侧升降速度反馈
                input.lift_right_torque,                    // 右侧升降力矩反馈
                rmcs_msgs::DartMechanismCommand::UP,        // 升降方向
                settings.lift_target_velocity,              // 升降目标速度
                rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY,    // 退出模式
                settings.lift_stall_velocity_threshold,     // 堵转速度阈值
                settings.lift_stall_torque_threshold,       // 堵转力矩阈值
                settings.lift_stall_confirm_ticks,          // 堵转确认帧数
                20000                                       // 超时 tick
                ));
    }
};

} // namespace rmcs_dart_guidance::manager
