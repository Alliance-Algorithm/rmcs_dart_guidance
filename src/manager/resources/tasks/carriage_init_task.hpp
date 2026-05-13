#pragma once

#include <functional>
#include <memory>
#include <numeric>
#include <optional>

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
// CarriageCalibrationTask
//   丝杆完整标定任务：重复三次低速下行堵转采样，并在每次采样后回退离开限位。
//   三次样本求平均后写回新的原点，再通过角度闭环动作移动到校准停靠位。
// ─────────────────────────────────────────────────────────────────────────────
class CarriageCalibrationTask : public Task {
public:
    CarriageCalibrationTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings, ManagerRuntimeState& runtime_state)
        : Task("carriage_init", "丝杆完整标定流程")
        , runtime_state_(runtime_state)
        , carriage_origin_angle_interface_(output.carriage_origin_angle) {
        constexpr double kCarriageCalibrationReturnEncoderAngle = 500000.0;
        constexpr std::size_t kCalibrationPassCount = 3;

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

        then(
            std::make_shared<CallbackAction>(
                "carriage_calibration_prepare", [this](CallbackAction&) {
                    runtime_state_.carriage_calibration_origin_samples.clear();
                    return ActionStatus::SUCCESS;
                }));

        for (std::size_t index = 0; index < kCalibrationPassCount; ++index) {
            then(make_calibration_action());

            then(std::make_shared<DelayAction>("delay", 100));

            then(
                std::make_shared<CallbackAction>(
                    "carriage_origin_record",                           // 动作名称
                    [this](CallbackAction& action) {
                        runtime_state_.carriage_calibration_origin_samples.push_back(
                            carriage_origin_angle_interface_);
                        if (const auto* logger = action.runtime_context().logger;
                            logger != nullptr) {
                            RCLCPP_INFO(
                                *logger,
                                "[CarriageCalibrationTask] sampled encoder origin[%zu]=%.6f",
                                runtime_state_.carriage_calibration_origin_samples.size(),
                                carriage_origin_angle_interface_);
                        }
                        return ActionStatus::SUCCESS;
                    }));                                                // 记录一次原点采样

            then(std::make_shared<DelayAction>("delay", 100));

            then(make_return_action());

            then(std::make_shared<DelayAction>("delay", 100));
        }

        then(
            std::make_shared<CallbackAction>(
                "carriage_origin_average_finalize",                     // 动作名称
                [this](CallbackAction& action) {
                    constexpr std::size_t kCalibrationPassCount = 3;
                    if (runtime_state_.carriage_calibration_origin_samples.size()
                        != kCalibrationPassCount) {
                        return action.fail_with(ActionFailureReason::INVALID_INPUT);
                    }
                    const double average_origin_angle =
                        std::accumulate(
                            runtime_state_.carriage_calibration_origin_samples.begin(),
                            runtime_state_.carriage_calibration_origin_samples.end(), 0.0)
                        / static_cast<double>(kCalibrationPassCount);
                    carriage_origin_angle_interface_ = average_origin_angle;
                    runtime_state_.carriage_power_cycle_origin_angle = average_origin_angle;
                    runtime_state_.carriage_calibration_origin_samples.clear();
                    return ActionStatus::SUCCESS;
                }));                                                    // 汇总采样并写回平均原点

        then(std::make_shared<DelayAction>("delay", 100));

        then(
            std::make_shared<CarriageAngleCloseLoopAction>(
                "carriage_calibration_park",                            // 动作名称
                output.carriage_command,                                // 丝杆命令接口
                output.carriage_target_velocity,                        // 丝杆目标速度接口
                output.carriage_target_angle,                           // 丝杆目标角度接口
                input.carriage_angle,                                   // 丝杆当前位置反馈
                input.carriage_origin_angle,                            // 丝杆原点角反馈
                rmcs_msgs::DartMechanismCommand::UP,                    // 回退方向
                settings.carriage_up_setting_velocity,                  // 闭环目标速度
                settings.carriage_calibration_parking_angle,            // 相对原点停靠角度
                settings.carriage_angle_allowable_error,                // 允许角度误差
                settings.carriage_min_run_ticks,                        // 最小运行 tick
                settings.carriage_timeout_ticks));
    }

    void on_enter() override {
        previous_origin_angle_ = runtime_state_.carriage_power_cycle_origin_angle;
        if (!previous_origin_angle_.has_value()) {
            previous_origin_angle_ = carriage_origin_angle_interface_;
        }
        Task::on_enter();
    }

    void on_exit() override {
        restore_previous_origin_if_needed(failure_info().reason != ActionFailureReason::NONE);
        runtime_state_.carriage_calibration_origin_samples.clear();
        Task::on_exit();
    }

    void on_cancel(ActionCancelReason reason) override {
        (void)reason;
        restore_previous_origin_if_needed(true);
        runtime_state_.carriage_calibration_origin_samples.clear();
        Task::on_cancel(reason);
    }

private:
    void restore_previous_origin_if_needed(const bool should_restore) {
        if (should_restore && previous_origin_angle_.has_value()) {
            carriage_origin_angle_interface_ = *previous_origin_angle_;
            runtime_state_.carriage_power_cycle_origin_angle = *previous_origin_angle_;
        }
    }

    ManagerRuntimeState& runtime_state_;
    double& carriage_origin_angle_interface_;
    std::optional<double> previous_origin_angle_;
};

} // namespace rmcs_dart_guidance::manager
