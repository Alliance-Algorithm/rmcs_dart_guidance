#pragma once

#include "manager/action/action_set.hpp"
#include "manager/action/belt_constant_velocity_move_action.hpp"
#include "manager/action/belt_deceleration_action.hpp"
#include "manager/action/belt_hold_torque_action.hpp"
#include "manager/action/belt_move_action.hpp"
#include "manager/action/belt_zero_calibration.hpp"
#include "manager/action/delay_action.hpp"
#include "manager/action/filling_lift_action.hpp"
#include "manager/action/trigger_control_action.hpp"
#include "manager/task/task.hpp"

#include <cmath>
#include <cstdint>
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
        double& belt_torque_offset, double& belt_error_gain, bool& belt_use_decel_pid,
        const double& left_belt_angle, const double& right_belt_angle,
        const double& left_belt_velocity, const double& right_belt_velocity,
        const double& left_belt_torque, const double& right_belt_torque, bool& trigger_lock_enable,
        rmcs_msgs::DartSliderStatus& lifting_command, const double& lifting_left_vel_fb,
        const double& lifting_right_vel_fb, double belt_down_distance, double belt_pulley_radius,
        double down_velocity, double torque_limit, double up_torque_limit,
        double down_hold_torque, double down_zero_velocity_threshold,
        uint64_t down_zero_confirm_ticks, uint64_t down_ramp_timeout_ticks,
        bool& belt_zero_calibration, bool require_lifting_up = true)
        : Task("cancel_launch", "取消发射") {

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
                +belt_down_distance,                // 目标距离（m，正值=下行）
                belt_pulley_radius,                 // 滑轮半径（m）
                down_velocity,                      // 运动速度（rad/s）
                torque_limit,                       // 扭矩限制（N⋅m）
                10000,                              // 超时帧数
                100,                                // 最小运行帧数
                0.5,                                // 位置到达容差（rad）- 增大容差
                0.3,                                // 堵转速度阈值（rad/s）
                200,                                // 堵转确认帧数
                0.20));                             // 下行最大距离限制（m，正值）

        // 步骤2：减速阶段（目标速度=0，加常态力矩偏移补偿负载，使用零速检测）
        then(
            std::make_shared<BeltDecelerationAction>(
                "belt_down_ramp_to_zero",     // 动作名称
                belt_target_velocity,         // 目标速度（输出）
                belt_torque_offset,           // 力矩偏移（输出）
                belt_error_gain,              // 误差增益（输出）
                belt_use_decel_pid,           // 使用减速PID（输出）
                left_belt_velocity,           // 左电机速度反馈（输入）
                right_belt_velocity,          // 右电机速度反馈（输入）
                left_belt_torque,             // 左电机力矩反馈（输入）
                right_belt_torque,            // 右电机力矩反馈（输入）
                0.0,                          // 目标速度（rad/s）
                down_hold_torque,             // 力矩偏移值（N⋅m）
                2.0,                          // 误差增益倍数
                true,                         // 启用堵转检测
                true,                         // 启用零速检测
                0.3,                          // 堵转速度阈值（rad/s）
                1.0,                          // 堵转扭矩阈值（N⋅m）
                down_zero_velocity_threshold, // 零速阈值（rad/s）
                50,                           // 堵转确认帧数
                down_zero_confirm_ticks,      // 零速确认帧数
                50,                           // 最小运行帧数
                down_ramp_timeout_ticks));    // 超时帧数

        // 步骤3：传送带保持高扭矩 + 解锁扳机 + (可选)升降上行并行
        auto parallel_hold_unlock_and_lift =
            std::make_shared<ActionSet>("hold_unlock_and_lift", ActionSet::Policy::ALL_SUCCESS);

        parallel_hold_unlock_and_lift
            ->also(
                std::make_shared<BeltHoldTorqueAction>(
                    "belt_hold_torque",      // 动作名称
                    belt_command,            // 传送带命令（输出）
                    belt_target_velocity,    // 目标速度（输出）
                    belt_hold_torque,        // 保持力矩（输出）
                    belt_wait_zero_velocity, // WAIT模式选择（输出）
                    belt_torque_offset,      // 力矩偏移（输出）
                    down_hold_torque,        // 保持力矩值（N⋅m）
                    2.5,                     // 力矩偏移值（N⋅m）- 取消发射不需要偏移
                    500))                    // 保持时长（ticks）
            .also(
                std::make_shared<TriggerControlAction>(
                    trigger_lock_enable,     // 扳机锁定使能（输出）
                    false,                   // 解锁（false）
                    500));                   // 等待释放完成帧数

        // 只有在需要时才添加升降上行动作（第二发及以后才需要）
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

        // 步骤4：同步带上行复位到机械限位
        then(
            std::make_shared<BeltMoveAction>(
                "belt_reset",                                 // 动作名称
                belt_command,                                 // 同步带目标状态（输出）
                belt_target_velocity,                         // 同步带目标速度（输出）
                belt_torque_limit,                            // 同步带力矩限制（输出）
                belt_hold_torque,                             // 同步带保持力矩（输出）
                belt_wait_zero_velocity,                      // Wait 时使用零速闭环还是保留力矩
                left_belt_velocity,                           // 左同步带反馈（输入）
                right_belt_velocity,                          // 右同步带反馈（输入）
                left_belt_torque,                             // 左同步带力矩（输入）
                right_belt_torque,                            // 右同步带力矩（输入）
                rmcs_msgs::DartSliderStatus::UP,              // 指令状态
                10,                                           // 设定速度
                up_torque_limit,                              // 设定力矩限制
                0.5,                                          // 设定保持力矩
                10000,                                        // 超时帧数
                1.0,                                          // 堵转速度阈值
                0.5,                                          // 堵转力矩阈值
                100,                                          // 堵转确认帧数
                50,                                           // 最短运行帧数
                BeltMoveAction::ExitMode::WAIT_ZERO_VELOCITY, // 退出模式
                false));                                      // 超时返回失败

        then(std::make_shared<DelayAction>("stabilize_wait", 50));

        then(std::make_shared<ZeroCalibrationAction>(belt_zero_calibration));
    }
};

} // namespace rmcs_dart_guidance::manager
