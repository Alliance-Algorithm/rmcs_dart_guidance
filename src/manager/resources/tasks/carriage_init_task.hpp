#pragma once

#include <functional>
#include <memory>
#include <numeric>

#include <rclcpp/logging.hpp>

#include "manager/core/runtime/action.hpp"
#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/carriage_control_action.hpp"
#include "manager/resources/actions/delay_action.hpp"

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// CallbackAction
//   任务内联辅助动作：把一次性的 lambda 回调包装成 IAction，便于在 ActionSequence
//   中插入轻量的数据记录、状态汇总或校验逻辑，而无需单独定义新类。
// ─────────────────────────────────────────────────────────────────────────────
class CallbackAction : public IAction {
public:
    using Callback = std::function<ActionStatus(CallbackAction&)>;

    CallbackAction(std::string name, Callback callback)
        : IAction(std::move(name))
        , callback_(std::move(callback)) {}

    ActionStatus update() override { return callback_(*this); }

    ActionStatus fail_with(ActionFailureReason reason) { return fail(reason); }

private:
    Callback callback_;
};

// ─────────────────────────────────────────────────────────────────────────────
// CarriageInitTask
//   丝杆初始化采样任务：先低速下行直到堵转以记录一组原点，再延时后回退到安全位置。
//   该任务会把本次测得的 origin 追加到 runtime_state 中，供后续平均收敛步骤使用。
// ─────────────────────────────────────────────────────────────────────────────
class CarriageInitTask : public Task {
public:
    CarriageInitTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings, ManagerRuntimeState& runtime_state)
        : Task("carriage_init", "丝杆低速堵转初始化") {
        constexpr double kCarriageCalibrationReturnEncoderAngle = 778240.0;
        auto carriage_origin_angle_interface = &output.carriage_origin_angle;

        auto make_calibration_action = [&input, &output, &settings]() {
            return std::make_shared<CarriageControlAction>(
                "carriage_init",                                        // 动作名称
                output.carriage_command,                                // 丝杆命令接口
                output.carriage_target_velocity,                        // 丝杆目标速度接口
                output.carriage_origin_angle,                           // 丝杆原点角接口
                output.force_max_torque_override,                       // 力矩上限覆盖接口
                input.carriage_angle,                                   // 丝杆当前位置反馈
                input.carriage_velocity,                                // 丝杆速度反馈
                input.carriage_torque,                                  // 丝杆力矩反馈
                rmcs_msgs::DartMechanismCommand::DOWN,                  // 初始化方向
                settings.carriage_calibration_velocity,                 // 初始化目标速度
                settings.carriage_calibration_max_torque,               // 初始化力矩上限
                settings.carriage_calibration_stall_velocity_threshold, // 堵转速度阈值
                settings.carriage_calibration_stall_torque_threshold,   // 堵转力矩阈值
                settings.carriage_calibration_stall_confirm_ticks);
        };

        auto make_return_action = [&input, &output, &settings,
                                   kCarriageCalibrationReturnEncoderAngle]() {
            return std::make_shared<CarriageTravelAction>(
                "carriage_travel",                                      // 动作名称
                output.carriage_command,                                // 丝杆命令接口
                output.carriage_target_velocity,                        // 丝杆目标速度接口
                input.carriage_angle,                                   // 丝杆当前位置反馈
                input.carriage_origin_angle,                            // 丝杆原点角反馈
                input.carriage_velocity,                                // 丝杆速度反馈
                input.carriage_torque,                                  // 丝杆力矩反馈
                rmcs_msgs::DartMechanismCommand::UP,                    // 回退方向
                settings.carriage_up_setting_velocity,                  // 回退目标速度
                kCarriageCalibrationReturnEncoderAngle,                 // 回退目标位移
                settings.carriage_stall_velocity_threshold,             // 堵转速度阈值
                settings.carriage_stall_torque_threshold,               // 堵转力矩阈值
                settings.carriage_stall_confirm_ticks,                  // 堵转确认帧数
                CarriageTravelAction::TravelReferenceMode::CURRENT_ANGLE);
        };

        then(make_calibration_action());
        then(std::make_shared<DelayAction>(
            "delay",              // 动作名称
            100                   // 延时 tick
            ));
        then(
            std::make_shared<CallbackAction>(
                "carriage_origin_record",                  // 动作名称
                [&runtime_state, carriage_origin_angle_interface](CallbackAction&) {
                    runtime_state.carriage_calibration_origin_samples.push_back(
                        *carriage_origin_angle_interface);
                    return ActionStatus::SUCCESS;
                }));                                       // 记录一次原点采样
        then(std::make_shared<DelayAction>(
            "delay",              // 动作名称
            100                   // 延时 tick
            ));
        then(make_return_action());
        then(std::make_shared<DelayAction>(
            "delay",              // 动作名称
            100                   // 延时 tick
            ));
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// CarriageInitFinalizeTask
//   丝杆初始化收敛任务：对前面多次采样得到的 origin 求平均，写回运行时原点，
//   然后通过角度闭环动作把丝杆移动到目标待机位置。若采样次数不足则返回
//   INVALID_INPUT。
// ─────────────────────────────────────────────────────────────────────────────
class CarriageInitFinalizeTask : public Task {
public:
    CarriageInitFinalizeTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings, ManagerRuntimeState& runtime_state)
        : Task("carriage_init_finalize", "丝杆初始化平均值收敛") {
        auto carriage_origin_angle_interface = &output.carriage_origin_angle;

        auto make_return_close_loop_action = [&input, &output, &settings]() {
            return std::make_shared<CarriageAngleCloseLoopAction>(
                "carriage_angle_close_loop",              // 动作名称
                output.carriage_command,                  // 丝杆命令接口
                output.carriage_target_velocity,          // 丝杆目标速度接口
                output.carriage_target_angle,             // 丝杆目标角度接口
                input.carriage_angle,                     // 丝杆当前位置反馈
                input.carriage_origin_angle,              // 丝杆原点角反馈
                rmcs_msgs::DartMechanismCommand::UP,      // 回退方向
                settings.carriage_up_setting_velocity,    // 闭环目标速度
                settings.carriage_down_travel_angle,      // 相对原点目标角度
                settings.carriage_angle_allowable_error,  // 允许角度误差
                settings.carriage_min_run_ticks,          // 最小运行 tick
                settings.carriage_timeout_ticks);
        };

        then(
            std::make_shared<CallbackAction>(
                "carriage_origin_average_finalize",        // 动作名称
                [&runtime_state, carriage_origin_angle_interface](CallbackAction& action) {
                    constexpr std::size_t kCalibrationPassCount = 3;
                    if (runtime_state.carriage_calibration_origin_samples.size()
                        != kCalibrationPassCount) {
                        return action.fail_with(ActionFailureReason::INVALID_INPUT);
                    }
                    const double average_origin_angle =
                        std::accumulate(
                            runtime_state.carriage_calibration_origin_samples.begin(),
                            runtime_state.carriage_calibration_origin_samples.end(), 0.0)
                        / static_cast<double>(kCalibrationPassCount);
                    *carriage_origin_angle_interface = average_origin_angle;
                    runtime_state.carriage_power_cycle_origin_angle = average_origin_angle;
                    runtime_state.carriage_calibration_origin_samples.clear();
                    return ActionStatus::SUCCESS;
                }));                                       // 汇总采样并写回平均原点
        then(std::make_shared<DelayAction>(
            "delay",              // 动作名称
            100                   // 延时 tick
            ));
        then(make_return_close_loop_action());
    }
};

} // namespace rmcs_dart_guidance::manager
