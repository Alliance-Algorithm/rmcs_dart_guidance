#pragma once

#include <memory>
#include <string>

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/carriage_control_action.hpp"

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// CarriageTravelTask
//   丝杆运动任务：先执行开环定程移动，再追加一个角度闭环动作做末端收敛。
//   默认构造用于发射滑台下行，也可通过扩展构造函数复用为任意方向和目标角度的
//   丝杆位移任务。
// ─────────────────────────────────────────────────────────────────────────────
class CarriageTravelTask : public Task {
public:
    CarriageTravelTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings)
        : CarriageTravelTask(
              "carriage_travel",                        // 任务名称
              "发射滑台运动",                           // 任务描述
              input,                                    // 管理器输入上下文
              output,                                   // 管理器输出上下文
              rmcs_msgs::DartMechanismCommand::DOWN,    // 丝杆默认运动方向
              settings.carriage_down_setting_velocity,  // 丝杆默认运动速度
              settings.carriage_down_travel_angle,      // 丝杆默认运动角度
              settings) {}                              // 管理器参数配置

    CarriageTravelTask(
        std::string name, std::string description, const ManagerInputContext& input,
        ManagerOutputContext& output, rmcs_msgs::DartMechanismCommand command_setting,
        double velocity_setting, double travel_angle_setting, const ManagerSettings& settings)
        : Task(std::move(name), std::move(description)) {
        then(
            std::make_shared<CarriageTravelAction>(
                "carriage_travel",                          // 动作名称
                output.carriage_command,                    // 丝杆命令接口
                output.carriage_target_velocity,            // 丝杆目标速度接口
                input.carriage_angle,                       // 丝杆当前位置反馈
                input.carriage_origin_angle,                // 丝杆原点角反馈
                input.carriage_velocity,                    // 丝杆速度反馈
                input.carriage_torque,                      // 丝杆力矩反馈
                command_setting,                            // 丝杆运动方向
                velocity_setting,                           // 开环运动速度
                travel_angle_setting,                       // 目标位移角度
                settings.carriage_stall_velocity_threshold, // 堵转速度阈值
                settings.carriage_stall_torque_threshold,   // 堵转力矩阈值
                settings.carriage_stall_confirm_ticks));    // 堵转确认帧数
        then(
            std::make_shared<CarriageAngleCloseLoopAction>(
                "carriage_angle_close_loop",              // 动作名称
                output.carriage_command,                  // 丝杆命令接口
                output.carriage_target_velocity,          // 丝杆目标速度接口
                output.carriage_target_angle,             // 丝杆目标角度接口
                input.carriage_angle,                     // 丝杆当前位置反馈
                input.carriage_origin_angle,              // 丝杆原点角反馈
                command_setting,                          // 闭环运动方向
                velocity_setting,                         // 闭环速度上限
                travel_angle_setting,                     // 相对原点目标角度
                settings.carriage_angle_allowable_error,  // 允许角度误差
                settings.carriage_min_run_ticks,          // 最小运行 tick
                settings.carriage_timeout_ticks));        // 闭环超时 tick
    }
};

} // namespace rmcs_dart_guidance::manager
