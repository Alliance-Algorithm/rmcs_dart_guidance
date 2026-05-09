#pragma once

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/belt_control_action.hpp"
#include "manager/resources/actions/filling_lift_action.hpp"
#include "manager/resources/actions/trigger_control_action.hpp"
#include "rmcs_msgs/dart_mechanism_command.hpp"
#include "rmcs_msgs/dart_motor_exit_mode.hpp"
#include "rmcs_msgs/dart_servo_command.hpp"

#include <memory>

namespace rmcs_dart_guidance::manager {

class LaunchPreparationTask : public Task {
public:
    LaunchPreparationTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings, const ManagerRuntimeState& runtime_state)
        : Task("launch_prepare", "滑块发射准备") {
        if (runtime_state.fire_count > 0) {
            then(
                std::make_shared<BeltTravelAction>(
                    "belt_down_travel_1",                      // 动作名称
                    output.belt_command,                       // 同步带命令接口
                    output.belt_target_velocity,               // 同步带目标速度接口
                    output.belt_exit_mode,                     // 电机退出状态接口
                    input.belt_left_angle,
                    input.belt_left_velocity,                  // 左电机速度反馈
                    input.belt_left_torque,                    // 左电机力矩反馈
                    input.belt_right_angle,
                    input.belt_right_velocity,                 // 右电机速度反馈
                    input.belt_right_torque,                   // 右电机力矩反馈
                    rmcs_msgs::DartMechanismCommand::DOWN,     // 同步带命令设置
                    settings.belt_down_setting_velocity * 1.8, // 同步带目标速度设置
                    rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY,   // 电机退出模式设置
                    settings.belt_down_travel_angle * 3.3,
                    20000                                      // 超时时间 ms
                    ));
        } else {
            then(
                std::make_shared<BeltTravelAction>(
                    "belt_down_travel_1",                      // 动作名称
                    output.belt_command,                       // 同步带命令接口
                    output.belt_target_velocity,               // 同步带目标速度接口
                    output.belt_exit_mode,                     // 电机退出状态接口
                    input.belt_left_angle,
                    input.belt_left_velocity,                  // 左电机速度反馈
                    input.belt_left_torque,                    // 左电机力矩反馈
                    input.belt_right_angle,
                    input.belt_right_velocity,                 // 右电机速度反馈
                    input.belt_right_torque,                   // 右电机力矩反馈
                    rmcs_msgs::DartMechanismCommand::DOWN,     // 同步带命令设置
                    settings.belt_down_setting_velocity * 1.0, // 同步带目标速度设置
                    rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY,   // 电机退出模式设置
                    settings.belt_down_travel_angle * 3.3,
                    20000                                      // 超时时间 ms
                    ));
        }
        then(
            std::make_shared<BeltTravelAction>(
                "belt_down_travel_2",                          // 动作名称
                output.belt_command,                           // 同步带命令接口
                output.belt_target_velocity,                   // 同步带目标速度接口
                output.belt_exit_mode,                         // 电机退出状态接口
                input.belt_left_angle,
                input.belt_left_velocity,                      // 左电机速度反馈
                input.belt_left_torque,                        // 左电机力矩反馈
                input.belt_right_angle,
                input.belt_right_velocity,                     // 右电机速度反馈
                input.belt_right_torque,                       // 右电机力矩反馈
                rmcs_msgs::DartMechanismCommand::DOWN,         // 同步带命令设置
                settings.belt_down_setting_velocity * 0.8,     // 同步带目标速度设置
                rmcs_msgs::ExitMode::WAIT_HOLD_TORQUE,         // 电机退出模式设置
                settings.belt_down_travel_angle * 0.5,
                10000                                          // 超时时间 ms
                ));

        if (runtime_state.fire_count > 0) {
            then(
                std::make_shared<TriggerControlAction>(
                    "trigger_lock",                            //
                    output.trigger_command,                    //
                    rmcs_msgs::DartServoCommand::LOCK,         //
                    100                                        //
                    ));
            then(
                std::make_shared<FillingLiftAction>(
                    "filling_lift_down",                       // 动作名称
                    output.lifting_command,                    // 升降命令接口
                    output.lift_target_velocity,               // 升降目标速度接口
                    output.lift_exit_mode,                     // 电机退出状态接口
                    input.lift_left_velocity,                  // 左电机速度反馈
                    input.lift_left_torque,                    // 左电机力矩反馈
                    input.lift_right_velocity,                 // 右电机速度反馈
                    input.lift_right_torque,                   // 右电机力矩反馈
                    rmcs_msgs::DartMechanismCommand::DOWN,     // 升降命令设置
                    settings.lift_target_velocity,             // 同步带目标速度设置
                    rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY,   // 电机退出模式设置
                    settings.lift_stall_velocity_threshold,    // 堵转速度阈值
                    settings.lift_stall_torque_threshold,      // 堵转力矩阈值
                    settings.lift_stall_confirm_ticks,         // 堵转确认帧数
                    20000                                      // 超时时间 ms
                    ));
        } else {
            then(
                std::make_shared<TriggerControlAction>(
                    "trigger_lock",                            //
                    output.trigger_command,                    //
                    rmcs_msgs::DartServoCommand::LOCK,         //
                    1000                                       //
                    ));
        }

        // KEEP allows the second segment to take over without clearing the belt command.
        then(
            std::make_shared<BeltTravelAction>(
                "belt_up_travel_1",                         // 动作名称
                output.belt_command,                        // 同步带命令接口
                output.belt_target_velocity,                // 同步带目标速度接口
                output.belt_exit_mode,                      // 电机退出状态接口
                input.belt_left_angle,                      // 左电机角度反馈
                input.belt_left_velocity,                   // 左电机速度反馈
                input.belt_left_torque,                     // 左电机力矩反馈
                input.belt_right_angle,                     // 右电机角度反馈
                input.belt_right_velocity,                  // 右电机速度反馈
                input.belt_right_torque,                    // 右电机力矩反馈
                rmcs_msgs::DartMechanismCommand::UP,        // 同步带命令设置
                settings.belt_up_setting_velocity * 0.5,    // 同步带目标速度设置
                rmcs_msgs::ExitMode::KEEP,                  // 电机退出模式设置
                settings.belt_up_travel_angle,              // 第一段上行角度
                20000                                       // 超时时间 ms
                ));

        then(
            std::make_shared<BeltTravelAction>(
                "belt_up_travel_2",                         // 动作名称
                output.belt_command,                        // 同步带命令接口
                output.belt_target_velocity,                // 同步带目标速度接口
                output.belt_exit_mode,                      // 电机退出状态接口
                input.belt_left_angle,                      // 左电机角度反馈
                input.belt_left_velocity,                   // 左电机速度反馈
                input.belt_left_torque,                     // 左电机力矩反馈
                input.belt_right_angle,                     // 右电机角度反馈
                input.belt_right_velocity,                  // 右电机速度反馈
                input.belt_right_torque,                    // 右电机力矩反馈
                rmcs_msgs::DartMechanismCommand::UP,        // 同步带命令设置
                settings.belt_up_setting_velocity * 2,      // 同步带目标速度设置
                rmcs_msgs::ExitMode::KEEP,                  // 电机退出模式设置
                settings.belt_up_travel_angle * 1.3,        // 第二段上行角度
                20000                                       // 超时时间 ms
                ));

        then(
            std::make_shared<BeltControlAction>(
                "belt_up_stall",                            // 动作名称
                output.belt_command,                        // 同步带命令接口
                output.belt_target_velocity,                // 同步带目标速度接口
                output.belt_exit_mode,                      // 电机退出状态接口
                output.belt_max_torque_override,            // 电机力矩上限覆盖接口
                input.belt_left_velocity,                   // 左电机速度反馈
                input.belt_left_torque,                     // 左电机力矩反馈
                input.belt_right_velocity,                  // 右电机速度反馈
                input.belt_right_torque,                    // 右电机力矩反馈
                rmcs_msgs::DartMechanismCommand::UP,        // 同步带命令设置
                settings.belt_up_setting_velocity * 0.3,    // 同步带目标速度设置
                rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY,    // 电机退出模式设置
                settings.belt_stall_velocity_threshold,     // 堵转速度阈值
                settings.belt_stall_torque_threshold * 0.5, // 堵转力矩阈值
                settings.belt_stall_confirm_ticks,          // 堵转确认帧数
                20000,                                      // 超时时间 ms
                settings.belt_init_max_torque               // 力矩上限
                ));

        // then(
        //     std::make_shared<ForceControlAction>(
        //         "force_close_loop",                          //
        //         output.force_error,                          //
        //         input.force_sensor_ch1,                      //
        //         input.force_sensor_ch2,                      //
        //         settings.force_setpoint,                     //
        //         settings.force_allowable_error,              //
        //         20000                                        //
        //         ));
    }
};

} // namespace rmcs_dart_guidance::manager
