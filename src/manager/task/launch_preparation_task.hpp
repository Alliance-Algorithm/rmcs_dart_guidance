#pragma once

#include "manager/action/action_set.hpp"
#include "manager/action/belt_constant_velocity_move_action.hpp"
#include "manager/action/belt_hold_torque_action.hpp"
#include "manager/action/belt_zero_calibration.hpp"
#include "manager/action/delay_action.hpp"
#include "manager/action/filling_lift_action.hpp"
#include "manager/action/force_screw_calibration_action.hpp"
#include "manager/action/trigger_control_action.hpp"
#include "manager/task/task.hpp"

#include <cmath>
#include <cstdint>
#include <memory>

#include <rclcpp/logger.hpp>
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
        bool& trigger_lock_enable, double belt_down_distance, double down_velocity,
        bool require_lifting_down, rmcs_msgs::DartSliderStatus& lifting_command,
        const double& lifting_left_vel_fb, const double& lifting_right_vel_fb,
        bool& belt_zero_calibration, double& force_screw_velocity, const int& current_force_ch1,
        const int& current_force_ch2, int force_channel, double target_force,
        bool enable_force_calibration, double force_tolerance, uint64_t force_settle_ticks,
        uint64_t force_timeout_ticks, double force_kp, double force_ki, double force_kd,
        bool is_first_shot = false)
        : Task("launch_preparation", "发射准备（传送带下行 + 扳机锁定 + 上行复位）") {

        // 步骤1：传送带匀速下行到目标位置（使用速度控制+多圈角度反馈）
        // 注意：target_distance为正值表示下行（角度增大方向）
        then(
            std::make_shared<BeltConstantVelocityMoveAction>(
                "belt_move_down_constant_velocity",        // 动作名称
                belt_command,                              // 速度模式方向命令（输出）
                belt_target_velocity,                      // 目标速度（输出）
                belt_torque_offset,                        // 力矩偏移（输出）
                belt_torque_limit,                         // 扭矩限幅（输出）
                0.0,                                       // 力矩偏移值
                left_belt_angle,                           // 左电机角度反馈（输入）
                right_belt_angle,                          // 右电机角度反馈（输入）
                left_belt_velocity,                        // 左电机速度反馈（输入）
                right_belt_velocity,                       // 右电机速度反馈（输入）
                +belt_down_distance - 0.1,                 // 目标距离（m，正值=下行）
                down_velocity,                             // 运动速度（rad/s）
                10,                                        // 扭矩限制（N⋅m）
                10000                                      // 超时帧数
                ));

        auto down_and_hold_ = std::make_shared<ActionSequence>("down_and_hold");
        down_and_hold_
            ->then(
                std::make_shared<BeltConstantVelocityMoveAction>(
                    "belt_slowly_down",                    // 动作名称
                    belt_command,                          // 速度模式方向命令（输出）
                    belt_target_velocity,                  // 目标速度（输出）
                    belt_torque_offset,                    // 力矩偏移（输出）
                    belt_torque_limit,                     // 扭矩限幅（输出）
                    0.0,                                   // 力矩偏移值
                    left_belt_angle,                       // 左电机角度反馈（输入）
                    right_belt_angle,                      // 右电机角度反馈（输入）
                    left_belt_velocity,                    // 左电机速度反馈（输入）
                    right_belt_velocity,                   // 右电机速度反馈（输入）
                    +0.1,                                  // 目标距离（m，正值=下行）
                    8,                                     // 运动速度（rad/s）
                    5,                                     // 扭矩限制（N⋅m）
                    10000                                  // 超时帧数
                    ))
            .then(
                std::make_shared<BeltHoldTorqueAction>(
                    "belt_hold_torque",                    // 动作名称
                    belt_command,                          // 传送带命令（输出）
                    belt_target_velocity,                  // 目标速度（输出）
                    belt_hold_torque,                      // 保持力矩（输出）
                    belt_wait_zero_velocity,               // WAIT模式选择（输出）
                    belt_torque_offset,                    // 力矩偏移（输出）
                    5.0,                                   // 保持力矩值（N⋅m）
                    2000));

        if (require_lifting_down) {
            auto parallel_hold_lock_and_lift =
                std::make_shared<ActionSet>("hold_lock_and_lift", ActionSet::Policy::ALL_SUCCESS);

            parallel_hold_lock_and_lift->also(down_and_hold_)
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

            parallel_hold_and_lock->also(down_and_hold_)
                .also(
                    std::make_shared<TriggerControlAction>(
                        trigger_lock_enable, // 扳机锁定使能（输出）
                        true,                // 锁定（true）
                        500));               // 等待锁定完成帧数
            then(parallel_hold_and_lock);
        }

        then(
            std::make_shared<BeltConstantVelocityMoveAction>(
                "belt_up_first_stage",       // 动作名称
                belt_command,                // 速度模式方向命令（输出）
                belt_target_velocity,        // 目标速度（输出）
                belt_torque_offset,          // 力矩偏移（输出）
                belt_torque_limit,           // 扭矩限幅（输出）
                1.0,                         // 力矩偏移值
                left_belt_angle,             // 左电机角度反馈（输入）
                right_belt_angle,            // 右电机角度反馈（输入）
                left_belt_velocity,          // 左电机速度反馈（输入）
                right_belt_velocity,         // 右电机速度反馈（输入）
                -0.05,                       // 目标距离
                10.0,                        // 快速（rad/s）
                5.0,                         // 扭矩限制（N⋅m）
                1000));

        then(
            std::make_shared<BeltConstantVelocityMoveAction>(
                "belt_up_second_stage",      // 动作名称
                belt_command,                // 速度模式方向命令（输出）
                belt_target_velocity,        // 目标速度（输出）
                belt_torque_offset,          // 力矩偏移（输出）
                belt_torque_limit,           // 扭矩限幅（输出）
                0.0,                         // 力矩偏移
                left_belt_angle,             // 左电机角度反馈（输入）
                right_belt_angle,            // 右电机角度反馈（输入）
                left_belt_velocity,          // 左电机速度反馈（输入）
                right_belt_velocity,         // 右电机速度反馈（输入）
                -0.65,                       // 目标距离
                20.0,                        // 快速（rad/s）
                3.0,                         // 扭矩限制（N⋅m）
                2000));

        then(
            std::make_shared<BeltConstantVelocityMoveAction>(
                "belt_up_third_stage",       // 动作名称
                belt_command,                // 速度模式方向命令（输出）
                belt_target_velocity,        // 目标速度（输出）
                belt_torque_offset,          // 力矩偏移（输出）
                belt_torque_limit,           // 扭矩限幅（输出）
                0.0,                         // 力矩偏移值（无）
                left_belt_angle,             // 左电机角度反馈（输入）
                right_belt_angle,            // 右电机角度反馈（输入）
                left_belt_velocity,          // 左电机速度反馈（输入）
                right_belt_velocity,         // 右电机速度反馈（输入）
                -0.75,                       // 目标距离）
                12.0,                        // 快速（rad/s）
                0.5,                         // 扭矩限制（N⋅m）
                1000));

        then(std::make_shared<DelayAction>("stabilize_wait", 50));

        then(std::make_shared<ZeroCalibrationAction>(belt_zero_calibration));

        if (enable_force_calibration) {
            double target_force_value = is_first_shot ? 16000.0 : target_force;
            double tolerance_value = is_first_shot ? 0.5 : force_tolerance;

            then(
                std::make_shared<ForceScrewCalibrationAction>(
                    "force_screw_calibration", force_screw_velocity, current_force_ch1,
                    current_force_ch2, force_channel, target_force_value, tolerance_value,
                    force_settle_ticks, force_timeout_ticks, force_kp, force_ki, force_kd, 5.0));
        }
    }
};
} // namespace rmcs_dart_guidance::manager