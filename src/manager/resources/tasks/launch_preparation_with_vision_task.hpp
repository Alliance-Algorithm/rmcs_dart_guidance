#pragma once

#include <memory>

#include "manager/core/runtime/action_set.hpp"
#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/trigger_carriage_position_aim_action.hpp"
#include "manager/resources/actions/vision_aim_action.hpp"
#include "manager/resources/tasks/launch_preparation_task.hpp"

namespace rmcs_dart_guidance::manager {

class LaunchPreparationWithVisionTask : public Task {
public:
    LaunchPreparationWithVisionTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings, const VisionAimProfileProvider& profile_provider,
        const ManagerRuntimeState& runtime_state)
        : Task("launch_prepare_with_vision", "视觉辅助发射准备") {
        auto action_set = std::make_shared<ActionSet>(
            "launch_prepare_with_vision_set", ActionSet::Policy::ALL_SUCCESS);

        action_set->also(
            std::make_shared<LaunchPreparationTask>(input, output, settings, runtime_state));
        // action_set->also(
        //     std::make_shared<VisionAimAction>(
        //         "vision_aim", input.current_target, input.tracking, input.target_seq,
        //         output.angle_error_vector, profile_provider, runtime_state.fire_count));
        action_set->also(
            std::make_shared<TriggerCarriagePositionAimAction>(
                "trigger_carriage_position_aim", output.carriage_command,
                output.carriage_target_velocity, output.carriage_target_angle, input.carriage_angle,
                input.carriage_origin_angle, profile_provider, runtime_state.fire_count,
                settings.carriage_down_setting_velocity, settings.carriage_up_setting_velocity,
                settings.carriage_angle_allowable_error, settings.carriage_timeout_ticks));

        then(action_set);
    }
};

} // namespace rmcs_dart_guidance::manager
