#pragma once

#include "manager/core/runtime/action_sequence.hpp"
#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/chassis_leveling_action.hpp"

#include <memory>
#include <string>

namespace rmcs_dart_guidance::manager {

namespace {

inline LevelingWheelFeedback make_leveling_wheel_feedback(const ManagerInputContext& input) {
    return LevelingWheelFeedback{
        MotorFeedbackView{ input.leveling_front_left_velocity,  input.leveling_front_left_torque},
        MotorFeedbackView{input.leveling_front_right_velocity, input.leveling_front_right_torque},
        MotorFeedbackView{  input.leveling_rear_left_velocity,   input.leveling_rear_left_torque},
        MotorFeedbackView{ input.leveling_rear_right_velocity,  input.leveling_rear_right_torque},
    };
}

inline LevelingMonitorParameters
    make_leveling_parameters(const ManagerSettings& settings, const uint64_t timeout_ticks) {
    return LevelingMonitorParameters{
        settings.chassis_leveling_stall_velocity_threshold,
        settings.chassis_leveling_stall_torque_threshold,
        settings.chassis_leveling_stall_confirm_ticks,
        settings.chassis_leveling_min_run_ticks,
        settings.chassis_leveling_angle_allowable_error,
        settings.chassis_leveling_angle_confirm_ticks,
        timeout_ticks,
    };
}

} // namespace

class LegacyChassisLevelingTask : public Task {
public:
    LegacyChassisLevelingTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings)
        : Task("chassis_leveling_legacy", "底盘调平（adjust）") {
        const auto wheel_feedback = make_leveling_wheel_feedback(input);
        const auto axis_parameters =
            make_leveling_parameters(settings, settings.chassis_leveling_axis_timeout_ticks);

        auto make_leveling_round = [&](const int round_index) {
            auto round = std::make_shared<ActionSequence>(
                "legacy_roll_pitch_leveling_" + std::to_string(round_index));
            round->then(
                std::make_shared<LegacyRollLevelingAction>(
                    "legacy_roll_leveling_" + std::to_string(round_index),
                    output.chassis_leveling_phase, input.roll_angle, wheel_feedback,
                    axis_parameters));
            round->then(
                std::make_shared<LegacyPitchLevelingAction>(
                    "legacy_pitch_leveling_" + std::to_string(round_index),
                    output.chassis_leveling_phase, input.pitch_angle, wheel_feedback,
                    axis_parameters));
            return round;
        };

        then(make_leveling_round(1));
        then(make_leveling_round(2));
        then(make_leveling_round(3));
    }
};

class ThreeStageChassisLevelingTask : public Task {
public:
    ThreeStageChassisLevelingTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings)
        : Task("chassis_leveling_three_stage", "底盘调平（三步式）") {
        const auto wheel_feedback = make_leveling_wheel_feedback(input);
        const auto contact_parameters =
            make_leveling_parameters(settings, settings.chassis_leveling_contact_timeout_ticks);
        const auto axis_parameters =
            make_leveling_parameters(settings, settings.chassis_leveling_axis_timeout_ticks);

        then(
            std::make_shared<StageContactLevelingAction>(
                "stage_contact_leveling", output.chassis_leveling_phase, wheel_feedback,
                contact_parameters));
        then(
            std::make_shared<StageRollLevelingAction>(
                "stage_roll_leveling", output.chassis_leveling_phase, input.roll_angle,
                wheel_feedback, axis_parameters));
        then(
            std::make_shared<StagePitchLevelingAction>(
                "stage_pitch_leveling", output.chassis_leveling_phase, input.pitch_angle,
                wheel_feedback, axis_parameters));
    }
};

using ChassisLevelingTask = ThreeStageChassisLevelingTask;

} // namespace rmcs_dart_guidance::manager
