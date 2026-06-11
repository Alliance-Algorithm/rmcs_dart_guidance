#pragma once

#include "manager/core/runtime/action_sequence.hpp"
#include "manager/core/runtime/action_set.hpp"
#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/belt_control_action.hpp"
#include "manager/resources/actions/chassis_leveling_action.hpp"
#include "manager/resources/actions/filling_lift_action.hpp"
#include <memory>

namespace rmcs_dart_guidance::manager {

class DartInitTask : public Task {
public:
    DartInitTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings)
        : Task("dart_init", "飞镖机构复位") {

        auto make_leveling_round = [&](int round_index) -> std::shared_ptr<ActionSequence> {
            auto round = std::make_shared<ActionSequence>(
                "roll_pitch_leveling_" + std::to_string(round_index));

            round->then(
                std::make_shared<RollLevelingAction>(
                    "roll_leveling_" + std::to_string(round_index),
                    output.chassis_roll_leveling_flag, input.roll_angle,
                    input.leveling_front_left_velocity, input.leveling_rear_left_velocity,
                    input.leveling_front_left_torque, input.leveling_rear_left_torque));

            round->then(
                std::make_shared<PitchLevelingAction>(
                    "pitch_leveling_" + std::to_string(round_index),
                    output.chassis_pitch_leveling_flag, input.pitch_angle,
                    input.leveling_front_left_velocity, input.leveling_front_left_torque,
                    input.leveling_front_right_velocity, input.leveling_front_right_torque));

            return round;
        };

        auto chassis_leveling = std::make_shared<ActionSequence>("chassis_leveling");
        chassis_leveling->then(make_leveling_round(1));
        chassis_leveling->then(make_leveling_round(2));
        chassis_leveling->then(make_leveling_round(3));

        auto action_set = std::make_shared<ActionSet>(
            "dart_init",                                     // 动作组名称
            ActionSet::Policy::ALL_SUCCESS                   // 所有子动作成功才算成功
        );
        action_set->also(
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
                settings.belt_init_stall_velocity_threshold, // 堵转速度阈值
                settings.belt_init_stall_torque_threshold,   // 堵转力矩阈值
                settings.belt_init_stall_confirm_ticks,      // 堵转确认帧数
                20000,                                       // 超时时间 ms
                settings.belt_init_max_torque                // 力矩上限
                )

        );

        action_set->also(
            std::make_shared<FillingLiftAction>(
                "filling_lift_up",                           // 动作名称
                output.lifting_command,                      // 升降命令接口
                output.lift_target_velocity,                 // 升降目标速度接口
                output.lift_exit_mode,                       // 升降退出模式接口
                input.lift_left_velocity,                    // 左侧升降速度反馈
                input.lift_left_torque,                      // 左侧升降力矩反馈
                input.lift_right_velocity,                   // 右侧升降速度反馈
                input.lift_right_torque,                     // 右侧升降力矩反馈
                rmcs_msgs::DartMechanismCommand::UP,         // 升降方向
                settings.lift_target_velocity,               // 升降目标速度
                rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY,     // 退出模式
                settings.lift_stall_velocity_threshold,      // 堵转速度阈值
                settings.lift_stall_torque_threshold,        // 堵转力矩阈值
                settings.lift_stall_confirm_ticks,           // 堵转确认帧数
                20000                                        // 超时 tick
                ));

        action_set->also(chassis_leveling);

        then(action_set);
    }
};

} // namespace rmcs_dart_guidance::manager
