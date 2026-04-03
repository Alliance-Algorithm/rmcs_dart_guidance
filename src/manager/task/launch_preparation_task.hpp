#pragma once

#include "manager/action/action_set.hpp"
#include "manager/action/belt_constant_velocity_move_action.hpp"
#include "manager/action/belt_deceleration_with_stall_action.hpp"
#include "manager/action/belt_hold_torque_action.hpp"
#include "manager/action/belt_move_action.hpp"
#include "manager/action/belt_pid_deceleration_action.hpp"
#include "manager/action/belt_zero_calibration.hpp"
#include "manager/action/delay_action.hpp"
#include "manager/action/filling_lift_action.hpp"
#include "manager/action/force_screw_calibration_action.hpp"
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
        bool& belt_zero_calibration, double belt_up_distance, double up_velocity,
        double up_decel_target_velocity, double up_decel_torque_offset,
        double up_stall_velocity_threshold, uint64_t up_stall_confirm_ticks,
        uint64_t up_stall_min_run_ticks, uint64_t up_decel_timeout_ticks,
        double& force_screw_velocity, const int& current_force, double target_force,
        bool enable_force_calibration, double force_tolerance, uint64_t force_settle_ticks,
        uint64_t force_timeout_ticks, double force_kp, double force_ki, double force_kd,
        double force_max_velocity, bool is_first_shot = false)
        : Task("launch_preparation", "发射准备（传送带下行 + 扳机锁定 + 上行复位）") {

        // 步骤1：传送带匀速下行到目标位置（使用速度控制+多圈角度反馈）
        // 注意：target_distance为正值表示下行（角度增大方向）
        then(
            std::make_shared<BeltConstantVelocityMoveAction>(
                "belt_move_down_constant_velocity", // 动作名称
                belt_command,                       // 速度模式方向命令（输出）
                belt_target_velocity,               // 目标速度（输出）
                belt_torque_offset,                 // 力矩偏移（输出）
                belt_torque_limit,                  // 扭矩限幅（输出）
                0.0,                                // 力矩偏移值
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
                0.80));                             // 下行最大距离限制（m，正值）

        // 步骤3：PID减速阶段（目标速度=0，加常态力矩偏移补偿负载）
        then(
            std::make_shared<BeltPIDDecelerationAction>(
                "belt_pid_deceleration",      // 动作名称
                belt_target_velocity,         // 目标速度（输出，设为0）
                belt_torque_offset,           // 力矩偏移（输出）
                left_belt_velocity,           // 左电机速度反馈（输入）
                right_belt_velocity,          // 右电机速度反馈（输入）
                down_torque_offset,           // 力矩偏移值（N⋅m）
                down_zero_velocity_threshold, // 零速阈值（rad/s）
                100,                          // 零速确认帧数
                1000));                       // 超时帧数

        // 步骤3：根据是否第一发选择不同的并行动作
        if (is_first_shot) {
            // 第一发：传送带保持高扭矩 + 扳机锁定并行（无升降下行）
            auto parallel_hold_and_lock =
                std::make_shared<ActionSet>("hold_and_lock", ActionSet::Policy::ALL_SUCCESS);

            parallel_hold_and_lock
                ->also(
                    std::make_shared<TriggerControlAction>(
                        trigger_lock_enable,          // 扳机锁定使能（输出）
                        true,                         // 锁定（true）
                        750))
                .also(
                    std::make_shared<BeltPIDDecelerationAction>(
                        "belt_pid_deceleration",      // 动作名称
                        belt_target_velocity,         // 目标速度（输出，设为0）
                        belt_torque_offset,           // 力矩偏移（输出）
                        left_belt_velocity,           // 左电机速度反馈（输入）
                        right_belt_velocity,          // 右电机速度反馈（输入）
                        down_torque_offset,           // 力矩偏移值（N⋅m）
                        down_zero_velocity_threshold, // 零速阈值（rad/s）
                        10,                           // 零速确认帧数
                        3000));                       // 超时帧数
            then(parallel_hold_and_lock);

            then(
                std::make_shared<BeltHoldTorqueAction>(
                    "belt_hold_torque",               // 动作名称
                    belt_command,                     // 传送带命令（输出）
                    belt_target_velocity,             // 目标速度（输出）
                    belt_hold_torque,                 // 保持力矩（输出）
                    belt_wait_zero_velocity,          // WAIT模式选择（输出）
                    belt_torque_offset,               // 力矩偏移（输出）
                    down_hold_torque,                 // 保持力矩值（N⋅m）
                    down_torque_offset,               // 力矩偏移值（N⋅m）
                    500));                            // 保持时长（ticks）

        } else if (require_lifting_down) {
            // 第2-4发：传送带保持高扭矩 + 升降平台下行 + 扳机锁定并行
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
                        1000))                   // 保持时长（ticks）
                .also(
                    std::make_shared<TriggerControlAction>(
                        trigger_lock_enable,     // 扳机锁定使能（输出）
                        true,                    // 锁定（true）
                        1000));                  // 等待锁定完成帧数
            then(parallel_hold_and_lock);
        }

        // 步骤4：传送带上行到目标位置（位置控制）
        // 目标位置略低于下行起点（比如 -0.65m，而下行是 0.70m）
        then(
            std::make_shared<BeltConstantVelocityMoveAction>(
                "belt_reset_up_position", // 动作名称
                belt_command,             // 速度模式方向命令（输出）
                belt_target_velocity,     // 目标速度（输出）
                belt_torque_offset,       // 力矩偏移（输出）
                belt_torque_limit,        // 扭矩限幅（输出）
                5.0,                      // 力矩偏移值
                left_belt_angle,          // 左电机角度反馈（输入）
                right_belt_angle,         // 右电机角度反馈（输入）
                left_belt_velocity,       // 左电机速度反馈（输入）
                right_belt_velocity,      // 右电机速度反馈（输入）
                -belt_up_distance,        // 目标距离（m，负值=上行）
                belt_pulley_radius,       // 滑轮半径（m）
                up_velocity,              // 运动速度（rad/s）
                up_torque_limit,          // 扭矩限制（N⋅m）
                15000,                    // 超时帧数
                50,                       // 最小运行帧数
                0.5,                      // 位置到达容差（rad）
                0.3,                      // 堵转速度阈值（rad/s）
                100,                      // 堵转确认帧数
                0.80));                   // 下行最大距离限制（m，正值）

        // 步骤5：减速并监测堵转（堵转标志成功，作为下次下行起点）
        then(
            std::make_shared<BeltDecelerationWithStallAction>(
                "belt_decel_and_stall", // 动作名称
                belt_target_velocity,   // 目标速度（输出）
                belt_torque_offset,     // 力矩偏移（输出）
                left_belt_velocity,     // 左电机速度反馈（输入）
                right_belt_velocity,    // 右电机速度反馈（输入）
                left_belt_torque,       // 左电机力矩反馈（输入）
                right_belt_torque,      // 右电机力矩反馈（输入）
                2.0,                    // 目标速度（rad/s）
                up_decel_torque_offset, // 力矩偏移（N⋅m）
                0.5,                    // 堵转速度阈值（rad/s）
                0.5,                    // 堵转扭矩阈值（N⋅m）
                100,                    // 堵转确认帧数
                10,                     // 最小运行帧数
                5000));                 // 超时帧数

        then(std::make_shared<DelayAction>("stabilize_wait", 50));

        then(std::make_shared<ZeroCalibrationAction>(belt_zero_calibration));

        // 力矩闭环：第一发使用固定目标力9525，后续发使用上次记录的力值
        // if (enable_force_calibration) {
        //     double target_force_value = is_first_shot ? 11300.0 : target_force;
        //     double tolerance_value = is_first_shot ? 3.0 : force_tolerance;

        //     then(
        //         std::make_shared<ForceScrewCalibrationAction>(
        //             "force_screw_calibration", // 动作名称
        //             force_screw_velocity,      // 丝杆电机速度（输出）
        //             current_force,             // 当前力传感器读数（输入）
        //             target_force_value,        // 目标力值（第一发9025，后续用记录值）
        //             tolerance_value,           // 力容差（第一发±30，后续用配置值）
        //             force_settle_ticks,        // 稳定确认帧数
        //             force_timeout_ticks,       // 超时帧数
        //             force_kp,                  // PID参数Kp
        //             force_ki,                  // PID参数Ki
        //             force_kd,                  // PID参数Kd
        //             force_max_velocity));      // 最大速度限制
        // }
    }
};

} // namespace rmcs_dart_guidance::manager
