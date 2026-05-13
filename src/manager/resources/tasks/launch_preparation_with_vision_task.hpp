#pragma once

#include <memory>
#include <optional>
#include <string>

#include "manager/core/runtime/action.hpp"
#include "manager/core/runtime/action_set.hpp"
#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/belt_control_action.hpp"
#include "manager/resources/actions/filling_lift_action.hpp"
#include "manager/resources/actions/trigger_carriage_position_aim_action.hpp"
#include "manager/resources/actions/trigger_control_action.hpp"
#include "manager/resources/actions/vision_aim_action.hpp"
#include "manager/resources/vision_aim_profile_provider.hpp"
#include "rmcs_msgs/dart_mechanism_command.hpp"
#include "rmcs_msgs/dart_motor_exit_mode.hpp"
#include "rmcs_msgs/dart_servo_command.hpp"

namespace rmcs_dart_guidance::manager {

class ConfigurationFailureAction : public IAction {
public:
    ConfigurationFailureAction(std::string name, std::string message)
        : IAction(std::move(name))
        , message_(std::move(message)) {}

    ActionStatus update() override {
        if (!message_.empty() && runtime_context().logger != nullptr) {
            RCLCPP_ERROR(*runtime_context().logger, "[ConfigurationFailureAction] %s", message_.c_str());
        }
        return fail(ActionFailureReason::CONFIGURATION_ERROR);
    }

private:
    std::string message_;
};

class LaunchPreparationVisionMechanicalTask : public Task {
public:
    LaunchPreparationVisionMechanicalTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings, const VisionAimProfileProvider& profile_provider,
        const ManagerRuntimeState& runtime_state)
        : Task("launch_prepare_with_vision_mechanical", "视觉版发射准备机械分支") {
        const auto active_profile = profile_provider.resolve(runtime_state.fire_count);
        if (!active_profile.has_value()) {
            const std::string error_message =
                profile_provider.valid()
                    ? "missing vision_aim shot profile for fire_count="
                          + std::to_string(runtime_state.fire_count)
                    : profile_provider.error_message();
            then(std::make_shared<ConfigurationFailureAction>(
                "launch_prepare_with_vision_missing_profile", error_message));
            return;
        }

        const bool is_followup_fire = runtime_state.fire_count > 0;
        const bool use_interference_relief =
            is_followup_fire
            && static_cast<double>(active_profile->trigger_carriage_position)
                   > settings.carriage_lift_down_limit;

        append_belt_down_stage(input, output, settings, is_followup_fire);

        if (use_interference_relief) {
            append_interference_relief_stage(input, output, settings);
        } else {
            append_standard_mid_stage(input, output, settings, is_followup_fire);
        }

        append_belt_up_stage(input, output, settings);
    }

private:
    void append_belt_down_stage(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings, bool is_followup_fire) {
        then(
            std::make_shared<BeltTravelAction>(
                "belt_down_travel_1", output.belt_command, output.belt_target_velocity,
                output.belt_exit_mode, input.belt_left_angle, input.belt_left_velocity,
                input.belt_left_torque, input.belt_right_angle, input.belt_right_velocity,
                input.belt_right_torque, rmcs_msgs::DartMechanismCommand::DOWN,
                settings.belt_down_setting_velocity * (is_followup_fire ? 1.8 : 1.0),
                rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY,
                settings.belt_down_travel_angle * 3.3, 20000));

        then(
            std::make_shared<BeltTravelAction>(
                "belt_down_travel_2", output.belt_command, output.belt_target_velocity,
                output.belt_exit_mode, input.belt_left_angle, input.belt_left_velocity,
                input.belt_left_torque, input.belt_right_angle, input.belt_right_velocity,
                input.belt_right_torque, rmcs_msgs::DartMechanismCommand::DOWN,
                settings.belt_down_setting_velocity * 1.0, rmcs_msgs::ExitMode::WAIT_HOLD_TORQUE,
                settings.belt_down_travel_angle * 0.5, 10000));
    }

    void append_standard_mid_stage(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings, bool is_followup_fire) {
        if (is_followup_fire) {
            then(std::make_shared<TriggerControlAction>(
                "trigger_lock", output.trigger_command, rmcs_msgs::DartServoCommand::LOCK, 100));
            then(std::make_shared<FillingLiftAction>(
                "filling_lift_down", output.lifting_command, output.lift_target_velocity,
                output.lift_exit_mode, input.lift_left_velocity, input.lift_left_torque,
                input.lift_right_velocity, input.lift_right_torque,
                rmcs_msgs::DartMechanismCommand::DOWN, settings.lift_target_velocity,
                rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY,
                settings.lift_stall_velocity_threshold, settings.lift_stall_torque_threshold,
                settings.lift_stall_confirm_ticks, 20000));
            return;
        }

        then(std::make_shared<TriggerControlAction>(
            "trigger_lock", output.trigger_command, rmcs_msgs::DartServoCommand::LOCK, 1000));
    }

    void append_interference_relief_stage(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings) {
        then(std::make_shared<FillingLiftAction>(
            "filling_lift_down", output.lifting_command, output.lift_target_velocity,
            output.lift_exit_mode, input.lift_left_velocity, input.lift_left_torque,
            input.lift_right_velocity, input.lift_right_torque,
            rmcs_msgs::DartMechanismCommand::DOWN, settings.lift_target_velocity,
            rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY, settings.lift_stall_velocity_threshold,
            settings.lift_stall_torque_threshold, settings.lift_stall_confirm_ticks, 20000));

        then(std::make_shared<BeltTravelAction>(
            "belt_up_interference_relief", output.belt_command, output.belt_target_velocity,
            output.belt_exit_mode, input.belt_left_angle, input.belt_left_velocity,
            input.belt_left_torque, input.belt_right_angle, input.belt_right_velocity,
            input.belt_right_torque, rmcs_msgs::DartMechanismCommand::UP,
            settings.belt_up_setting_velocity * 0.5, rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY,
            settings.belt_interference_relief_travel_angle, 20000));

        then(std::make_shared<FillingLiftAction>(
            "filling_lift_up", output.lifting_command, output.lift_target_velocity,
            output.lift_exit_mode, input.lift_left_velocity, input.lift_left_torque,
            input.lift_right_velocity, input.lift_right_torque,
            rmcs_msgs::DartMechanismCommand::UP, settings.lift_target_velocity,
            rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY, settings.lift_stall_velocity_threshold,
            settings.lift_stall_torque_threshold, settings.lift_stall_confirm_ticks, 20000));

        then(std::make_shared<BeltTravelAction>(
            "belt_down_interference_relief", output.belt_command, output.belt_target_velocity,
            output.belt_exit_mode, input.belt_left_angle, input.belt_left_velocity,
            input.belt_left_torque, input.belt_right_angle, input.belt_right_velocity,
            input.belt_right_torque, rmcs_msgs::DartMechanismCommand::DOWN,
            settings.belt_down_setting_velocity * 1.0, rmcs_msgs::ExitMode::WAIT_HOLD_TORQUE,
            settings.belt_interference_relief_travel_angle, 20000));

        then(std::make_shared<TriggerControlAction>(
            "trigger_lock", output.trigger_command, rmcs_msgs::DartServoCommand::LOCK, 100));
    }

    void append_belt_up_stage(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings) {
        then(std::make_shared<BeltTravelAction>(
            "belt_up_travel_1", output.belt_command, output.belt_target_velocity,
            output.belt_exit_mode, input.belt_left_angle, input.belt_left_velocity,
            input.belt_left_torque, input.belt_right_angle, input.belt_right_velocity,
            input.belt_right_torque, rmcs_msgs::DartMechanismCommand::UP,
            settings.belt_up_setting_velocity * 0.5, rmcs_msgs::ExitMode::KEEP,
            settings.belt_up_travel_angle, 20000));

        then(std::make_shared<BeltTravelAction>(
            "belt_up_travel_2", output.belt_command, output.belt_target_velocity,
            output.belt_exit_mode, input.belt_left_angle, input.belt_left_velocity,
            input.belt_left_torque, input.belt_right_angle, input.belt_right_velocity,
            input.belt_right_torque, rmcs_msgs::DartMechanismCommand::UP,
            settings.belt_up_setting_velocity * 2.0, rmcs_msgs::ExitMode::KEEP,
            settings.belt_up_travel_angle * 1.3, 20000));

        then(std::make_shared<BeltControlAction>(
            "belt_up_stall", output.belt_command, output.belt_target_velocity,
            output.belt_exit_mode, output.belt_max_torque_override, input.belt_left_velocity,
            input.belt_left_torque, input.belt_right_velocity, input.belt_right_torque,
            rmcs_msgs::DartMechanismCommand::UP, settings.belt_up_setting_velocity * 0.3,
            rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY,
            settings.belt_stall_velocity_threshold, settings.belt_stall_torque_threshold * 0.5,
            settings.belt_stall_confirm_ticks, 20000, settings.belt_init_max_torque));
    }
};

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
            "launch_prepare_with_vision_set", ActionSet::Policy::ALL_SUCCESS);

        action_set->also(std::make_shared<LaunchPreparationVisionMechanicalTask>(
            input, output, settings, profile_provider, runtime_state));
        action_set->also(std::make_shared<VisionAimAction>(
            "vision_aim", input.current_target, input.tracking, input.target_seq,
            input.pitch_angle, output.angle_error_vector, profile_provider,
            runtime_state.fire_count));
        action_set->also(std::make_shared<TriggerCarriagePositionAimAction>(
            "trigger_carriage_position_aim", output.carriage_command,
            output.carriage_target_velocity, output.carriage_target_angle, input.carriage_angle,
            input.carriage_origin_angle, profile_provider, runtime_state.fire_count,
            settings.carriage_down_setting_velocity, settings.carriage_up_setting_velocity,
            settings.carriage_angle_allowable_error, settings.carriage_timeout_ticks));

        then(action_set);
    }
};

} // namespace rmcs_dart_guidance::manager
