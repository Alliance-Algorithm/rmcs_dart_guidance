#pragma once

#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

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
            RCLCPP_ERROR(
                *runtime_context().logger, "[ConfigurationFailureAction] %s", message_.c_str());
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
            const std::string error_message = profile_provider.valid()
                                                ? "missing vision_aim shot profile for fire_count="
                                                      + std::to_string(runtime_state.fire_count)
                                                : profile_provider.error_message();
            then(
                std::make_shared<ConfigurationFailureAction>(
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
        const double no_torque_limit = std::numeric_limits<double>::quiet_NaN();

        then(
            std::make_shared<BeltDisplacementPlanAction>(
                "belt_down_plan", output.belt_command, output.belt_target_velocity,
                output.belt_exit_mode, output.belt_max_torque_override, input.belt_left_angle,
                input.belt_right_angle, settings.belt_down_setting_velocity,
                std::vector<BeltDisplacementSwitchPoint>{
                    {
                        settings.belt_down_travel_angle * 3.3,
                        is_followup_fire ? 2.0 : 1.0,
                        rmcs_msgs::DartMechanismCommand::DOWN,
                        rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY,
                        no_torque_limit,
                    },
                    {
                        settings.belt_down_travel_angle * 3.8,
                        1.2,
                        rmcs_msgs::DartMechanismCommand::DOWN,
                        rmcs_msgs::ExitMode::WAIT_HOLD_TORQUE,
                        no_torque_limit,
                    },
                },
                20000));
    }

    void append_standard_mid_stage(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings, bool is_followup_fire) {
        if (is_followup_fire) {
            then(
                std::make_shared<TriggerControlAction>(
                    "trigger_lock", output.trigger_command, rmcs_msgs::DartServoCommand::LOCK, 10));
            then(
                std::make_shared<FillingLiftAction>(
                    "filling_lift_down", output.lifting_command, output.lift_target_velocity,
                    output.lift_exit_mode, input.lift_left_velocity, input.lift_left_torque,
                    input.lift_right_velocity, input.lift_right_torque,
                    rmcs_msgs::DartMechanismCommand::DOWN, settings.lift_target_velocity,
                    rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY, settings.lift_stall_velocity_threshold,
                    settings.lift_stall_torque_threshold, settings.lift_stall_confirm_ticks,
                    20000));
            return;
        }

        then(
            std::make_shared<TriggerControlAction>(
                "trigger_lock", output.trigger_command, rmcs_msgs::DartServoCommand::LOCK, 10));
    }

    void append_interference_relief_stage(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings) {
        const double no_torque_limit = std::numeric_limits<double>::quiet_NaN();

        then(
            std::make_shared<FillingLiftAction>(
                "filling_lift_down", output.lifting_command, output.lift_target_velocity,
                output.lift_exit_mode, input.lift_left_velocity, input.lift_left_torque,
                input.lift_right_velocity, input.lift_right_torque,
                rmcs_msgs::DartMechanismCommand::DOWN, settings.lift_target_velocity,
                rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY, settings.lift_stall_velocity_threshold,
                settings.lift_stall_torque_threshold, settings.lift_stall_confirm_ticks, 20000));

        then(
            std::make_shared<BeltDisplacementPlanAction>(
                "belt_interference_relief_plan", output.belt_command, output.belt_target_velocity,
                output.belt_exit_mode, output.belt_max_torque_override, input.belt_left_angle,
                input.belt_right_angle, settings.belt_up_setting_velocity,
                std::vector<BeltDisplacementSwitchPoint>{
                    {
                        settings.belt_interference_relief_travel_angle,
                        0.5,
                        rmcs_msgs::DartMechanismCommand::UP,
                        rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY,
                        no_torque_limit,
                    },
                },
                20000));

        then(
            std::make_shared<FillingLiftAction>(
                "filling_lift_up", output.lifting_command, output.lift_target_velocity,
                output.lift_exit_mode, input.lift_left_velocity, input.lift_left_torque,
                input.lift_right_velocity, input.lift_right_torque,
                rmcs_msgs::DartMechanismCommand::UP, settings.lift_target_velocity,
                rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY, settings.lift_stall_velocity_threshold,
                settings.lift_stall_torque_threshold, settings.lift_stall_confirm_ticks, 20000));

        then(
            std::make_shared<BeltControlAction>(
                "belt_down",                                // 动作名称
                output.belt_command,                        // 同步带命令接口
                output.belt_target_velocity,                // 同步带目标速度接口
                output.belt_exit_mode,                      // 电机退出状态接口
                output.belt_max_torque_override,            // 电机力矩上限覆盖接口
                input.belt_left_velocity,                   // 左电机速度反馈
                input.belt_left_torque,                     // 左电机力矩反馈
                input.belt_right_velocity,                  // 右电机速度反馈
                input.belt_right_torque,                    // 右电机力矩反馈
                rmcs_msgs::DartMechanismCommand::DOWN,      // 同步带命令设置
                settings.belt_down_setting_velocity * 1.55, // 同步带目标速度设置
                rmcs_msgs::ExitMode::WAIT_HOLD_TORQUE,      // 电机退出模式设置
                0.1,                                        // 堵转速度阈值
                4.5,                                        // 堵转力矩阈值
                200,                                        // 堵转确认帧数
                20000,                                      // 超时时间 ms
                5.5                                         // 力矩上限
                ));

        then(
            std::make_shared<TriggerControlAction>(
                "trigger_lock", output.trigger_command, rmcs_msgs::DartServoCommand::LOCK, 200));
    }

    void append_belt_up_stage(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings) {
        const double no_torque_limit = std::numeric_limits<double>::quiet_NaN();

        then(
            std::make_shared<BeltDisplacementPlanAction>(
                "belt_up_plan", output.belt_command, output.belt_target_velocity,
                output.belt_exit_mode, output.belt_max_torque_override, input.belt_left_angle,
                input.belt_right_angle, settings.belt_up_setting_velocity,
                std::vector<BeltDisplacementSwitchPoint>{
                    {
                        settings.belt_up_travel_angle,
                        0.5,
                        rmcs_msgs::DartMechanismCommand::UP,
                        rmcs_msgs::ExitMode::KEEP,
                        no_torque_limit,
                    },
                    {
                        settings.belt_up_travel_angle * 2.3,
                        2.0,
                        rmcs_msgs::DartMechanismCommand::UP,
                        rmcs_msgs::ExitMode::KEEP,
                        no_torque_limit,
                    },
                },
                20000));

        then(
            std::make_shared<BeltControlAction>(
                "belt_up_stall", output.belt_command, output.belt_target_velocity,
                output.belt_exit_mode, output.belt_max_torque_override, input.belt_left_velocity,
                input.belt_left_torque, input.belt_right_velocity, input.belt_right_torque,
                rmcs_msgs::DartMechanismCommand::UP, settings.belt_up_setting_velocity * 0.3,
                rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY, settings.belt_stall_velocity_threshold,
                settings.belt_stall_torque_threshold * 0.5, settings.belt_stall_confirm_ticks,
                20000, settings.belt_init_max_torque));
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

        action_set->also(
            std::make_shared<LaunchPreparationVisionMechanicalTask>(
                input, output, settings, profile_provider, runtime_state));
        // action_set->also(
        //     std::make_shared<VisionAimAction>(
        //         "vision_aim", input.current_target, input.tracking, input.target_seq,
        //         input.pitch_angle, output.angle_error_vector, profile_provider,
        //         runtime_state.fire_count));
        // action_set->also(
        //     std::make_shared<TriggerCarriagePositionAimAction>(
        //         "trigger_carriage_position_aim", output.carriage_command,
        //         output.carriage_target_velocity, output.carriage_target_angle,
        //         input.carriage_angle, input.carriage_origin_angle, profile_provider,
        //         runtime_state.fire_count, settings.carriage_down_setting_velocity,
        //         settings.carriage_up_setting_velocity, settings.carriage_angle_allowable_error,
        //         settings.carriage_timeout_ticks));

        then(action_set);
    }
};

} // namespace rmcs_dart_guidance::manager
