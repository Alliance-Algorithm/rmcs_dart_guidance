#pragma once

#include "manager/action/action_set.hpp"
#include "manager/action/belt_move_action.hpp"
#include "manager/action/delay_action.hpp"
#include "manager/action/filling_lift_action.hpp"
#include "manager/action/trigger_control_action.hpp"
#include "manager/task/task.hpp"

#include <cstdint>
#include <memory>

#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

// LaunchPreparationTask — 将机构切换到发射准备态。
//   NORMAL:
//     1. 同步带下行到装填位
//     2. 短暂等待机构稳定
//     3. 扳机锁定与填装升降下行并行执行
//     4. 同步带上行复位
//   FIRST_FILL:
//     1. 填装升降上行
//     2. 同步带下行到装填位
//     3. 短暂等待机构稳定
//     4. 扳机锁定
//     5. 同步带上行复位
class LaunchPreparationTask : public Task {
public:
    enum class Mode {
        NORMAL,
        FIRST_FILL,
    };

    LaunchPreparationTask(
        rmcs_msgs::DartSliderStatus& belt_command, double& belt_target_velocity,
        double& belt_torque_limit, double& belt_hold_torque, bool& belt_wait_zero_velocity,
        const double& left_belt_velocity, const double& right_belt_velocity,
        const double& left_belt_torque, const double& right_belt_torque, bool& trigger_lock_enable,
        rmcs_msgs::DartSliderStatus& lifting_command, const double& lifting_left_vel_fb,
        const double& lifting_right_vel_fb, double lifting_stall_threshold,
        uint64_t lifting_stall_confirm_ticks, uint64_t lifting_stall_min_run_ticks,
        uint64_t lifting_stall_timeout_ticks, Mode mode = Mode::NORMAL)
        : Task("launch_preparation", "滑块发射准备") {

        // 在任务内部定义相关物理参数，避免从外部传参，让结构更整洁
        double torque_limit = 5.0;
        double hold_torque = 1.0;                  // Wait 时的保持力矩

        auto make_belt_move_down = [&]() {
            return std::make_shared<BeltMoveAction>(
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
                10.0,                              // 设定速度
                torque_limit,                      // 设定力矩限制
                hold_torque,                       // 设定保持力矩
                10000,                             // 超时帧数
                1.0,                               // 堵转速度阈值
                0.5,                               // 堵转力矩阈值
                100,                               // 堵转确认帧数
                50,                                // 最短运行帧数
                BeltMoveAction::ExitMode::WAIT_HOLD_TORQUE);
        };

        auto make_belt_wait = [&]() {
            return std::make_shared<DelayAction>(
                "belt_wait", // 动作名称
                50           // 等待帧数
            );
        };

        auto make_trigger_lock = [&]() {
            return std::make_shared<TriggerControlAction>(
                trigger_lock_enable, // 扳机锁定使能（输出）
                true,                // 锁定（true）
                1000                 // 等待锁定完成帧数
            );
        };

        auto make_filling_lift =
            [&](const char* name, rmcs_msgs::DartSliderStatus command) {
                return std::make_shared<FillingLiftAction>(
                    name,                            // 动作名称
                    lifting_command,                 // 升降指令（输出）
                    command,                         // 指令状态
                    lifting_left_vel_fb,             // 左升降电机速度反馈（输入）
                    lifting_right_vel_fb,            // 右升降电机速度反馈（输入）
                    lifting_stall_threshold,         // 堵转速度阈值
                    lifting_stall_confirm_ticks,     // 堵转确认帧数
                    lifting_stall_min_run_ticks,     // 最短运行帧数
                    lifting_stall_timeout_ticks      // 超时帧数
                );
            };

        auto make_belt_reset = [&]() {
            return std::make_shared<BeltMoveAction>(
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
                20.0,                              // 设定速度
                torque_limit,                      // 设定力矩限制
                hold_torque,                       // 设定保持力矩
                5000,                              // 超时帧数
                0.5,                               // 堵转速度阈值
                0.5,                               // 堵转力矩阈值
                200,                               // 堵转确认帧数
                50,                                // 最短运行帧数
                BeltMoveAction::ExitMode::WAIT_ZERO_VELOCITY);
        };

        if (mode == Mode::FIRST_FILL) {
            then(make_filling_lift("filling_lift_up", rmcs_msgs::DartSliderStatus::UP));
            then(make_belt_move_down());
            then(make_belt_wait());
            then(make_trigger_lock());
            then(make_belt_reset());
            return;
        }

        then(make_belt_move_down());
        then(make_belt_wait());

        auto parallel_prepare =
            std::make_shared<ActionSet>("parallel_prepare", ActionSet::Policy::ALL_SUCCESS);
        parallel_prepare
            ->also(make_trigger_lock())
            .also(make_filling_lift("filling_lift_down", rmcs_msgs::DartSliderStatus::DOWN));
        then(parallel_prepare);

        then(make_belt_reset());
    }
};

} // namespace rmcs_dart_guidance::manager
