#pragma once

#include "manager/action/belt_move_action.hpp"
#include "manager/action/delay_action.hpp"
#include "manager/action/filling_lift_action.hpp"
#include "manager/action/trigger_control_action.hpp"
#include "manager/task/task.hpp"

#include <cstdint>
#include <memory>

#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

// CancelLaunchTask — 取消当前发射流程并回到安全待机位：
//   1. 同步带下行到卸载位
//   2. 短暂等待机构稳定
//   3. 解锁扳机
//   4. 同步带上行复位
//   5. 填装升降上行回到初始位
class CancelLaunchTask : public Task {
public:
    CancelLaunchTask(
        rmcs_msgs::DartSliderStatus& belt_command, double& belt_target_velocity,
        double& belt_torque_limit, double& belt_hold_torque, bool& belt_wait_zero_velocity,
        const double& left_belt_velocity, const double& right_belt_velocity,
        const double& left_belt_torque, const double& right_belt_torque, bool& trigger_lock_enable,
        rmcs_msgs::DartSliderStatus& lifting_command, const double& lifting_left_vel_fb,
        const double& lifting_right_vel_fb, double lifting_stall_threshold,
        uint64_t lifting_stall_confirm_ticks, uint64_t lifting_stall_min_run_ticks,
        uint64_t lifting_stall_timeout_ticks)
        : Task("cancel_launch", "取消发射") {

        // 在任务内部定义相关物理参数，避免从外部传参，让结构更整洁
        double down_velocity = 10.0;
        double up_velocity = 10.0;
        double hold_torque = 1.0;                  // Wait 时的保持力矩

        then(
            std::make_shared<BeltMoveAction>(
                "belt_move_down",                  // 动作名称
                belt_command,                      // 同步带目标状态（输出）
                belt_target_velocity,              // 同步带目标速度（输出）
                belt_torque_limit,                 // 同步带力矩限制（输出）
                belt_hold_torque,                  // 同步带保持力矩（输出）
                belt_wait_zero_velocity,           // Wait 时使用零速闭环还是保留力矩
                left_belt_velocity,                // 左同步带反馈（输入）
                right_belt_velocity,               // 右同步带反馈（输入）
                left_belt_torque,                  // 左同步带力矩（输入）
                right_belt_torque,                 // 右同步带力矩（输入）
                rmcs_msgs::DartSliderStatus::DOWN, // 指令状态
                down_velocity,                     // 设定速度
                50.0,                              // 设定力矩限制
                hold_torque,                       // 设定保持力矩
                20000,                             // 超时帧数
                1.0,                               // 堵转速度阈值
                0.5,                               // 堵转力矩阈值
                100,                               // 堵转确认帧数
                50,                                // 最短运行帧数
                BeltMoveAction::ExitMode::WAIT_HOLD_TORQUE));

        then(
            std::make_shared<DelayAction>(
                "belt_wait",                       // 动作名称
                50                                 // 等待帧数
                ));

        then(
            std::make_shared<TriggerControlAction>(
                trigger_lock_enable,               // 扳机锁定使能（输出）
                false,                             // 解锁（false）
                1000                               // 等待释放完成帧数
                ));

        then(
            std::make_shared<BeltMoveAction>(
                "belt_reset",                      // 动作名称
                belt_command,                      // 同步带目标状态（输出）
                belt_target_velocity,              // 同步带目标速度（输出）
                belt_torque_limit,                 // 同步带力矩限制（输出）
                belt_hold_torque,                  // 同步带保持力矩（输出）
                belt_wait_zero_velocity,           // Wait 时使用零速闭环还是保留力矩
                left_belt_velocity,                // 左同步带反馈（输入）
                right_belt_velocity,               // 右同步带反馈（输入）
                left_belt_torque,                  // 左同步带力矩（输入）
                right_belt_torque,                 // 右同步带力矩（输入）
                rmcs_msgs::DartSliderStatus::UP,   // 指令状态
                up_velocity,                       // 设定速度
                25.0,                              // 设定力矩限制
                hold_torque,                       // 设定保持力矩
                20000,                             // 超时帧数
                1.0,                               // 堵转速度阈值
                0.5,                               // 堵转力矩阈值
                100,                               // 堵转确认帧数
                50,                                // 最短运行帧数
                BeltMoveAction::ExitMode::WAIT_ZERO_VELOCITY));

        then(
            std::make_shared<FillingLiftAction>(
                "filling_lift_up",                 // 动作名称
                lifting_command,                   // 升降指令（输出）
                rmcs_msgs::DartSliderStatus::UP,   // 指令状态
                lifting_left_vel_fb,               // 左升降电机速度反馈（输入）
                lifting_right_vel_fb,              // 右升降电机速度反馈（输入）
                lifting_stall_threshold,           // 堵转速度阈值
                lifting_stall_confirm_ticks,       // 堵转确认帧数
                lifting_stall_min_run_ticks,       // 最短运行帧数
                lifting_stall_timeout_ticks        // 超时帧数
                ));
    }
};

} // namespace rmcs_dart_guidance::manager
