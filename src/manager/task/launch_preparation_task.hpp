#pragma once

#include "manager/action/action_set.hpp"
#include "manager/action/belt_constant_velocity_move_action.hpp"
#include "manager/action/belt_hold_torque_action.hpp"
#include "manager/action/belt_move_action.hpp"
#include "manager/action/belt_pid_deceleration_action.hpp"
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
// LaunchPreparationTask — 每次发射前的准备动作（传送带下行+扳机锁定）
//   速度和扭矩限制由 manager 传入，不再硬编码
class LaunchPreparationTask : public Task {
public:
    LaunchPreparationTask(
        rmcs_msgs::DartSliderStatus& belt_command, double& belt_target_velocity,
        double& belt_torque_limit, double& belt_hold_torque, bool& belt_wait_zero_velocity,
        double& belt_torque_offset, const double& left_belt_angle, const double& right_belt_angle,
        const double& left_belt_velocity, const double& right_belt_velocity,
        const double& left_belt_torque, const double& right_belt_torque, bool& trigger_lock_enable,
        double belt_down_distance, double belt_pulley_radius, double down_velocity,
        double torque_limit, double up_torque_limit, uint64_t down_ramp_ticks,
        double down_torque_offset, double down_hold_torque, double down_zero_velocity_threshold,
        uint64_t down_zero_confirm_ticks, uint64_t down_ramp_timeout_ticks,
        bool require_lifting_down, rmcs_msgs::DartSliderStatus& lifting_command,
        const double& lifting_left_vel_fb, const double& lifting_right_vel_fb,
        bool& belt_zero_calibration)
        : Task("launch_preparation", "发射准备（传送带下行 + 扳机锁定 + 上行复位）") {

        // 步骤1：传送带匀速下行到目标位置（使用速度控制+多圈角度反馈）
        // 注意：target_distance为正值表示下行（角度增大方向）
        then(
            std::make_shared<BeltConstantVelocityMoveAction>(
                "belt_move_down_constant_velocity", // 动作名称
                belt_command,                       // 速度模式方向命令（输出）
                belt_target_velocity,               // 目标速度（输出）
                belt_torque_limit,                  // 扭矩限幅（输出）
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
                100,                                // 堵转确认帧数
                0.60));                             // 下行最大距离限制（m，正值）

        // 步骤2：PID减速阶段（目标速度=0，加常态力矩偏移补偿负载）
        // 必须等待速度真正降到零，否则保持阶段的常态力矩会导致疯转
        then(
            std::make_shared<BeltPIDDecelerationAction>(
                "belt_pid_deceleration",      // 动作名称
                belt_target_velocity,         // 目标速度（输出，设为0）
                belt_torque_offset,           // 力矩偏移（输出）
                left_belt_velocity,           // 左电机速度反馈（输入）
                right_belt_velocity,          // 右电机速度反馈（输入）
                down_torque_offset,           // 力矩偏移值（N⋅m）
                down_zero_velocity_threshold, // 零速阈值（rad/s）
                down_zero_confirm_ticks,      // 零速确认帧数
                down_ramp_timeout_ticks));    // 超时帧数

        if (require_lifting_down) {
            // 步骤3（2-4发）：传送带保持高扭矩 + 升降平台下行 + 扳机锁定并行
            auto parallel_hold_lock_and_lift =
                std::make_shared<ActionSet>("hold_lock_and_lift", ActionSet::Policy::ALL_SUCCESS);

            parallel_hold_lock_and_lift
                ->also(
                    std::make_shared<BeltHoldTorqueAction>(
                        "belt_hold_torque",                // 动作名称
                        belt_command,                      // 传送带命令（输出）
                        belt_target_velocity,              // 目标速度（输出）
                        belt_hold_torque,                  // 保持力矩（输出）
                        belt_wait_zero_velocity,           // WAIT模式选择（输出）
                        belt_torque_offset,                // 力矩偏移（输出）
                        down_hold_torque,                  // 保持力矩值（N⋅m）
                        down_torque_offset,                // 力矩偏移值（N⋅m）
                        1000))                             // 保持时长（ticks）
                .also(
                    std::make_shared<TriggerControlAction>(
                        trigger_lock_enable,               // 扳机锁定使能（输出）
                        true,                              // 锁定（true）
                        1000))                             // 等待锁定完成帧数
                .also(
                    std::make_shared<FillingLiftAction>(
                        "filling_lift_down",               // 动作名称
                        lifting_command,                   // 升降指令（输出）
                        rmcs_msgs::DartSliderStatus::DOWN, // 指令状态
                        lifting_left_vel_fb,               // 左升降电机速度反馈（输入）
                        lifting_right_vel_fb,              // 右升降电机速度反馈（输入）
                        0.1,                               // 堵转速度阈值（rad/s）
                        100,                               // 堵转确认帧数
                        500,                               // 最短运行帧数
                        1000));                            // 超时帧数
            then(parallel_hold_lock_and_lift);
        } else {
            // 步骤3（首发）：传送带保持高扭矩 + 扳机锁定并行
            auto parallel_hold_and_lock =
                std::make_shared<ActionSet>("hold_and_lock", ActionSet::Policy::ALL_SUCCESS);

            parallel_hold_and_lock
                ->also(
                    std::make_shared<BeltHoldTorqueAction>(
                        "belt_hold_torque",      // 动作名称
                        belt_command,            // 传送带命令（输出）
                        belt_target_velocity,    // 目标速度（输出）
                        belt_hold_torque,        // 保持力矩（输出）
                        belt_wait_zero_velocity, // WAIT模式选择（输出）
                        belt_torque_offset,      // 力矩偏移（输出）
                        down_hold_torque,        // 保持力矩值（N⋅m）
                        down_torque_offset,      // 力矩偏移值（N⋅m）
                        500))                    // 保持时长（ticks）
                .also(
                    std::make_shared<TriggerControlAction>(
                        trigger_lock_enable,     // 扳机锁定使能（输出）
                        true,                    // 锁定（true）
                        500));                   // 等待锁定完成帧数
            then(parallel_hold_and_lock);
        }

        // 步骤4：传送带上行到机械限位（速度控制 + 堵转检测）
        then(
            std::make_shared<BeltMoveAction>(
                "belt_reset_up",                              // 动作名称
                belt_command,                                 // 同步带目标状态（输出）
                belt_target_velocity,                         // 同步带目标速度（输出）
                belt_torque_limit,                            // 同步带力矩限制（输出）
                belt_hold_torque,                             // 同步带保持力矩（输出）
                belt_wait_zero_velocity,                      // Wait 时使用零速闭环还是保留力矩
                left_belt_velocity,                           // 左同步带反馈（输入）
                right_belt_velocity,                          // 右同步带反馈（输入）
                left_belt_torque,                             // 左同步带力矩（输入）
                right_belt_torque,                            // 右同步带力矩（输出）
                rmcs_msgs::DartSliderStatus::UP,              // 指令状态
                15,                                           // 设定速度（rad/s）
                up_torque_limit,                              // 设定力矩限制（N⋅m）
                0.5,                                          // 设定保持力矩（N⋅m）
                5000,                                         // 超时帧数
                0.5,                                          // 堵转速度阈值（rad/s）
                0.8,                                          // 堵转力矩阈值（N⋅m）
                100,                                          // 堵转确认帧数
                50,                                           // 最短运行帧数
                BeltMoveAction::ExitMode::WAIT_ZERO_VELOCITY, // 退出模式
                false));                                      // 超时返回失败

        then(std::make_shared<DelayAction>("stabilize_wait", 50));

        then(std::make_shared<ZeroCalibrationAction>(belt_zero_calibration));
    }
};

} // namespace rmcs_dart_guidance::manager
