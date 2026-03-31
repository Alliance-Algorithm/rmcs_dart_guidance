#pragma once

#include "manager/action/belt_move_action.hpp"
#include "manager/action/belt_zero_calibration.hpp"
#include "manager/action/delay_action.hpp"
#include "manager/task/task.hpp"
#include <memory>

#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

// SliderInitTask — 上电/恢复时执行一次同步带上行复位并校准零点：
//   1. 同步带上行到机械限位（堵转检测）
//   2. 短暂延时稳定
//   3. 发送零点校准命令
class SliderInitTask : public Task {
public:
    SliderInitTask(
        rmcs_msgs::DartSliderStatus& belt_command, double& belt_target_velocity,
        double& belt_torque_limit, double& belt_hold_torque, bool& belt_wait_zero_velocity,
        const double& left_belt_velocity, const double& right_belt_velocity,
        const double& left_belt_torque, const double& right_belt_torque,
        bool& belt_zero_calibration)
        : Task("slider_init", "传送带上行复位并校准零点") {

        // 步骤1：同步带上行到机械限位（堵转检测）
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
                right_belt_torque,                            // 右同步带力矩（输出）
                rmcs_msgs::DartSliderStatus::UP,              // 指令状态
                10,                                           // 设定速度
                1.0,                                          // 设定力矩限制
                0.5,                                          // 设定保持力矩
                3000,                                         // 超时帧数（可配置）
                1.0,                                          // 堵转速度阈值（rad/s）
                0.5,                                          // 堵转力矩阈值（N⋅m）
                100,                                          // 堵转确认帧数
                50,                                           // 最短运行帧数
                BeltMoveAction::ExitMode::WAIT_ZERO_VELOCITY, // 退出模式
                true)); // 超时也返回 SUCCESS（测试环境无接驳件）

        then(std::make_shared<DelayAction>("stabilize_wait", 50));

        then(std::make_shared<ZeroCalibrationAction>(belt_zero_calibration));
    }

private:
};

} // namespace rmcs_dart_guidance::manager
