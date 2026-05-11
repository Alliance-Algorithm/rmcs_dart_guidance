#pragma once

#include <memory>

#include "manager/core/runtime/action_set.hpp"
#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/trigger_carriage_position_aim_action.hpp"
#include "manager/resources/actions/vision_aim_action.hpp"
#include "manager/resources/tasks/launch_preparation_task.hpp"

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// LaunchPreparationWithVisionTask
//   视觉辅助发射准备任务：把机械发射准备流程与视觉瞄准流程放进同一个 ActionSet 中
//   并行执行，采用 ALL_SUCCESS 策略，只有两条分支都成功时任务才成功。
// ─────────────────────────────────────────────────────────────────────────────
class LaunchPreparationWithVisionTask : public Task {
public:
    LaunchPreparationWithVisionTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings, const VisionAimProfileProvider& profile_provider,
        const ManagerRuntimeState& runtime_state)
        : Task("launch_prepare_with_vision", "视觉辅助发射准备") {
        auto action_set = std::make_shared<ActionSet>(
            "launch_prepare_with_vision_set",            // 动作组名称
            ActionSet::Policy::ALL_SUCCESS               // 所有子动作成功才算成功
        );

        action_set->also(
            std::make_shared<LaunchPreparationTask>(
                input,                                   // 管理器输入上下文
                output,                                  // 管理器输出上下文
                settings,                                // 管理器参数配置
                runtime_state                            // 运行时状态
                ));
        action_set->also(
            std::make_shared<VisionAimAction>(
                "vision_aim", input.current_target, input.tracking, input.target_seq,
                input.pitch_angle, output.angle_error_vector, profile_provider,
                runtime_state.fire_count));
        action_set->also(
            std::make_shared<TriggerCarriagePositionAimAction>(
                "trigger_carriage_position_aim",         // 动作名称
                output.carriage_command,                 // 丝杆命令接口
                output.carriage_target_velocity,         // 丝杆目标速度接口
                output.carriage_target_angle,            // 丝杆目标角度接口
                input.carriage_angle,                    // 丝杆当前位置反馈
                input.carriage_origin_angle,             // 丝杆原点角反馈
                profile_provider,                        // 视觉 profile 提供器
                runtime_state.fire_count,                // 当前发射次数
                settings.carriage_down_setting_velocity, // 下行速度上限
                settings.carriage_up_setting_velocity,   // 上行速度上限
                settings.carriage_angle_allowable_error, // 允许角度误差
                settings.carriage_timeout_ticks));       // 超时 tick

        then(action_set);
    }
};

} // namespace rmcs_dart_guidance::manager
