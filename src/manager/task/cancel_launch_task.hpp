#pragma once

#include "manager/action/action_set.hpp"
#include "manager/action/belt_constant_velocity_move_action.hpp"
#include "manager/action/belt_hold_torque_action.hpp"
#include "manager/action/belt_zero_calibration.hpp"
#include "manager/action/delay_action.hpp"
#include "manager/action/filling_lift_action.hpp"
#include "manager/action/trigger_control_action.hpp"
#include "manager/task/task.hpp"

#include <cmath>
#include <memory>

#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

// CancelLaunchTask — 取消当前发射流程并回到安全待机位：
//   1. 同步带匀速下行到卸载位
//   2. 斜坡减速到零
//   3. 解锁扳机
//   4. 同步带上行复位
//   5. 填装升降上行回到初始位
class CancelLaunchTask : public Task {
public:
    CancelLaunchTask(
        rmcs_msgs::DartSliderStatus& belt_command, double& belt_target_velocity,
        double& belt_torque_limit, double& belt_hold_torque, bool& belt_wait_zero_velocity,
        double& belt_torque_offset, const double& left_belt_angle, const double& right_belt_angle,
        const double& left_belt_velocity, const double& right_belt_velocity,
        bool& trigger_lock_enable, rmcs_msgs::DartSliderStatus& lifting_command,
        const double& lifting_left_vel_fb, const double& lifting_right_vel_fb,
        double belt_down_distance, bool& belt_zero_calibration,
        double& force_screw_control_velocity, bool require_lifting_up = true)
        : Task("cancel_launch", "取消发射") {

        // 立即停止力丝杆电机（直接设置速度为0）
        force_screw_control_velocity = 0.0;

        // 步骤1：传送带匀速下行到卸载位（使用速度控制+多圈角度反馈）
        then(
            std::make_shared<BeltConstantVelocityMoveAction>(
                "belt_move_down_constant_velocity", // 动作名称
                belt_command,                       // 速度模式方向命令（输出）
                belt_target_velocity,               // 目标速度（输出）
                belt_torque_offset,                 // 力矩偏移（输出）
                belt_torque_limit,                  // 扭矩限幅（输出）
                0.0,
                left_belt_angle,                    // 左电机角度反馈（输入）
                right_belt_angle,                   // 右电机角度反馈（输入）
                left_belt_velocity,                 // 左电机速度反馈（输入）
                right_belt_velocity,                // 右电机速度反馈（输入）
                +belt_down_distance - 0.1,          // 目标距离（m，正值=下行）
                12,                                 // 运动速度（rad/s）
                10,                                 // 扭矩限制（N⋅m）
                10000));                            // 超时帧数

        auto down_and_hold_ = std::make_shared<ActionSequence>("down_and_hold");
        down_and_hold_
            ->then(
                std::make_shared<BeltConstantVelocityMoveAction>(
                    "belt_slowly_down",             // 动作名称
                    belt_command,                   // 速度模式方向命令（输出）
                    belt_target_velocity,           // 目标速度（输出）
                    belt_torque_offset,             // 力矩偏移（输出）
                    belt_torque_limit,              // 扭矩限幅（输出）
                    0.0,                            // 力矩偏移值
                    left_belt_angle,                // 左电机角度反馈（输入）
                    right_belt_angle,               // 右电机角度反馈（输入）
                    left_belt_velocity,             // 左电机速度反馈（输入）
                    right_belt_velocity,            // 右电机速度反馈（输入）
                    +0.1,                           // 目标距离（m，正值=下行）
                    8,                              // 运动速度（rad/s）
                    5,                              // 扭矩限制（N⋅m）
                    10000                           // 超时帧数
                    ))
            .then(
                std::make_shared<BeltHoldTorqueAction>(
                    "belt_hold_torque",             // 动作名称
                    belt_command,                   // 传送带命令（输出）
                    belt_target_velocity,           // 目标速度（输出）
                    belt_hold_torque,               // 保持力矩（输出）
                    belt_wait_zero_velocity,        // WAIT模式选择（输出）
                    belt_torque_offset,             // 力矩偏移（输出）
                    5.0,                            // 保持力矩值（N⋅m）
                    2000));

        // 步骤3：传送带保持高扭矩 + 解锁扳机 + (可选)升降上行并行
        auto parallel_hold_unlock_and_lift =
            std::make_shared<ActionSet>("hold_unlock_and_lift", ActionSet::Policy::ALL_SUCCESS);

        parallel_hold_unlock_and_lift->also(down_and_hold_)
            .also(std::make_shared<TriggerControlAction>(trigger_lock_enable, false, 500));

        if (require_lifting_up) {
            parallel_hold_unlock_and_lift->also(
                std::make_shared<FillingLiftAction>(
                    "filling_lift_up",               // 动作名称
                    lifting_command,                 // 升降指令（输出）
                    rmcs_msgs::DartSliderStatus::UP, // 指令状态
                    lifting_left_vel_fb,             // 左升降电机速度反馈（输入）
                    lifting_right_vel_fb,            // 右升降电机速度反馈（输入）
                    0.1,                             // 堵转速度阈值
                    100,                             // 堵转确认帧数
                    500,                             // 最短运行帧数
                    2000));                          // 超时帧数
        }

        then(parallel_hold_unlock_and_lift);

        then(
            std::make_shared<BeltConstantVelocityMoveAction>(
                "belt_up_first_stage",               // 动作名称
                belt_command,                        // 速度模式方向命令（输出）
                belt_target_velocity,                // 目标速度（输出）
                belt_torque_offset,                  // 力矩偏移（输出）
                belt_torque_limit,                   // 扭矩限幅（输出）
                1.0,                                 // 力矩偏移值
                left_belt_angle,                     // 左电机角度反馈（输入）
                right_belt_angle,                    // 右电机角度反馈（输入）
                left_belt_velocity,                  // 左电机速度反馈（输入）
                right_belt_velocity,                 // 右电机速度反馈（输入）
                -0.01,                               // 目标距离
                15.0,                                // 快速（rad/s）
                5.0,                                 // 扭矩限制（N⋅m）
                10000));

        then(
            std::make_shared<BeltConstantVelocityMoveAction>(
                "belt_up_second_stage",              // 动作名称
                belt_command,                        // 速度模式方向命令（输出）
                belt_target_velocity,                // 目标速度（输出）
                belt_torque_offset,                  // 力矩偏移（输出）
                belt_torque_limit,                   // 扭矩限幅（输出）
                0.0,                                 // 力矩偏移
                left_belt_angle,                     // 左电机角度反馈（输入）
                right_belt_angle,                    // 右电机角度反馈（输入）
                left_belt_velocity,                  // 左电机速度反馈（输入）
                right_belt_velocity,                 // 右电机速度反馈（输入）
                -0.65,                               // 目标距离
                20.0,                                // 快速（rad/s）
                3.0,                                 // 扭矩限制（N⋅m）
                10000));

        then(
            std::make_shared<BeltConstantVelocityMoveAction>(
                "belt_up_third_stage",               // 动作名称
                belt_command,                        // 速度模式方向命令（输出）
                belt_target_velocity,                // 目标速度（输出）
                belt_torque_offset,                  // 力矩偏移（输出）
                belt_torque_limit,                   // 扭矩限幅（输出）
                0.0,                                 // 力矩偏移值（无）
                left_belt_angle,                     // 左电机角度反馈（输入）
                right_belt_angle,                    // 右电机角度反馈（输入）
                left_belt_velocity,                  // 左电机速度反馈（输入）
                right_belt_velocity,                 // 右电机速度反馈（输入）
                -0.75,                               // 目标距离）
                15.0,                                // 快速（rad/s）
                0.8,                                 // 扭矩限制（N⋅m）
                10000));

        then(std::make_shared<DelayAction>("stabilize_wait", 50));

        then(std::make_shared<ZeroCalibrationAction>(belt_zero_calibration));
    }
};

} // namespace rmcs_dart_guidance::manager
