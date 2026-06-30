#pragma once

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/manual_control_action.hpp"

#include <memory>

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// ManualControlTask
//   手动控制任务：仅封装一个 ManualControlAction，把遥控器输入持续映射为同步带、
//   填装机构、扳机、角度误差、力度误差和调平电机手动速度设定输出，供人工直接接管
//   机构动作。
// ─────────────────────────────────────────────────────────────────────────────
class ManualControlTask : public Task {
public:
    ManualControlTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings)
        : Task("manual_control", "手动控制") {

        then(
            std::make_shared<ManualControlAction>(
                "manual_control",                                   // 动作名称
                input.remote_left_switch,                           // 左拨杆状态
                input.remote_right_switch,                          // 右拨杆状态
                input.remote_rotary_knob_switch,                    // 拨轮状态
                input.remote_left_joystick,                         // 左摇杆输入
                input.remote_right_joystick,                        // 右摇杆输入
                output.belt_command,                                // 同步带命令接口
                output.belt_target_velocity,                        // 同步带目标速度接口
                output.belt_exit_mode,                              // 同步带退出模式接口
                output.lifting_command,                             // 升降命令接口
                output.lift_target_velocity,                        // 升降目标速度接口
                output.lift_exit_mode,                              // 升降退出模式接口
                output.trigger_command,                             // 扳机命令接口
                output.force_error,                                 // 力度误差接口
                output.angle_error_vector,                          // 姿态误差接口
                settings.manual_angle_max_error,                    // 手动角度最大误差
                settings.manual_force_max_error,                    // 手动力度最大误差
                settings.belt_manual_setting_velocity,              // 手动同步带最大速度
                settings.lift_target_velocity,                      // 手动升降目标速度
                output.leveling_front_left_target_velocity,         // 左前调平速度设定
                output.leveling_front_right_target_velocity,        // 右前调平速度设定
                output.leveling_rear_left_target_velocity,          // 左后调平速度设定
                output.leveling_rear_right_target_velocity,         // 右后调平速度设定
                output.chassis_leveling_phase                       // 手动模式下强制退出自动调平
                ));
    }
};

} // namespace rmcs_dart_guidance::manager
