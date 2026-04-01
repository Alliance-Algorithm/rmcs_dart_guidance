#pragma once

#include "manager/action/filling_lift_action.hpp"
#include "manager/action/filling_limit_servo_action.hpp"
#include "manager/action/trigger_control_action.hpp"
#include "manager/task/task.hpp"

#include <cstdint>
#include <memory>

#include <rmcs_msgs/dart_limiting_servo_status.hpp>
#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

// FireAndPreloadTask — 执行一次发射并完成预装填：
//   1. 解锁扳机
//   2. 填装升降上行
//   3. 限位舵机执行预装填动作
class FireAndPreloadTask : public Task {
public:
    FireAndPreloadTask(
        bool& trigger_lock_enable, rmcs_msgs::DartSliderStatus& lifting_command,
        const double& lifting_left_vel_fb, const double& lifting_right_vel_fb,
        double lifting_stall_threshold, uint64_t lifting_stall_confirm_ticks,
        uint64_t lifting_stall_min_run_ticks, uint64_t lifting_stall_timeout_ticks,
        rmcs_msgs::DartLimitingServoStatus& limiting_command, uint64_t preload_fill_ticks,
        bool is_first_shot = false)
        : Task("fire", "发射并预装填") {

        // 第一发特殊处理：只解锁扳机，无需升降和预装填
        if (is_first_shot) {
            then(
                std::make_shared<TriggerControlAction>(
                    trigger_lock_enable,                      // 扳机锁定使能（输出）
                    false,                                    // 解锁（false）
                    500                                       // 等待释放完成帧数
                    ));
            return;
        } else {
            then(
                std::make_shared<TriggerControlAction>(
                    trigger_lock_enable,                      // 扳机锁定使能（输出）
                    false,                                    // 解锁（false）
                    500                                       // 等待释放完成帧数
                    ));

            then(
                std::make_shared<FillingLiftAction>(
                    "filling_lift_up",                        // 动作名称
                    lifting_command,                          // 升降指令（输出）
                    rmcs_msgs::DartSliderStatus::UP,          // 指令状态
                    lifting_left_vel_fb,                      // 左升降电机速度反馈（输入）
                    lifting_right_vel_fb,                     // 右升降电机速度反馈（输入）
                    lifting_stall_threshold,                  // 堵转速度阈值
                    lifting_stall_confirm_ticks,              // 堵转确认帧数
                    lifting_stall_min_run_ticks,              // 最短运行帧数
                    lifting_stall_timeout_ticks               // 超时帧数
                    ));

            then(
                std::make_shared<FillingLimitServoAction>(
                    limiting_command,                         // 限位舵机状态（输出）
                    rmcs_msgs::DartLimitingServoStatus::FREE, // 先释放
                    rmcs_msgs::DartLimitingServoStatus::LOCK, // 再锁回
                    preload_fill_ticks                        // 预装填持续帧数
                    ));
        }
    }
};

} // namespace rmcs_dart_guidance::manager
