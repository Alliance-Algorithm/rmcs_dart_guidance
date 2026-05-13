#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>

#include <rclcpp/logging.hpp>

#include "manager/core/runtime/action.hpp"
#include "manager/resources/vision_aim_profile_provider.hpp"
#include "rmcs_msgs/dart_mechanism_command.hpp"

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// TriggerCarriagePositionAimAction
//   发射位丝杆瞄准动作：根据当前发射次数读取 profile 中的触发滑台位置，换算出
//   相对原点的目标角度，并输出丝杆方向、速度限制和目标角度给下层控制器。
//   当当前角度进入允许误差范围时返回 SUCCESS；配置缺失或超时则返回相应失败原因。
// ─────────────────────────────────────────────────────────────────────────────
class TriggerCarriagePositionAimAction : public IAction {
public:
    TriggerCarriagePositionAimAction(
        std::string name, rmcs_msgs::DartMechanismCommand& carriage_command_interface,
        double& carriage_target_velocity_interface, double& carriage_target_angle_interface,
        const double& carriage_angle, const double& carriage_origin_angle,
        const VisionAimProfileProvider& profile_provider, uint32_t fire_count,
        double carriage_down_velocity, double carriage_up_velocity, double allowable_error,
        uint64_t timeout_ticks)
        : IAction(std::move(name))
        , carriage_command_(carriage_command_interface)
        , carriage_target_velocity_(carriage_target_velocity_interface)
        , carriage_target_angle_(carriage_target_angle_interface)
        , carriage_angle_(carriage_angle)
        , carriage_origin_angle_(carriage_origin_angle)
        , profile_provider_(profile_provider)
        , fire_count_(fire_count)
        , carriage_down_velocity_(carriage_down_velocity)
        , carriage_up_velocity_(carriage_up_velocity)
        , allowable_error_(allowable_error)
        , timeout_ticks_(timeout_ticks) {}

    void on_enter() override {
        configuration_error_message_.clear();
        active_profile_.reset();
        target_angle_ = std::numeric_limits<double>::quiet_NaN();
        target_velocity_limit_ = 0.0;
        command_ = rmcs_msgs::DartMechanismCommand::WAIT;

        active_profile_ = profile_provider_.resolve(fire_count_);
        if (!active_profile_.has_value()) {
            if (!profile_provider_.valid()) {
                configuration_error_message_ = profile_provider_.error_message();
            } else {
                configuration_error_message_ =
                    "missing vision_aim shot profile for fire_count=" + std::to_string(fire_count_);
            }
            apply_outputs();
            return;
        }

        target_angle_ = carriage_origin_angle_
                      + static_cast<double>(active_profile_->trigger_carriage_position);

        const double angle_error = target_angle_ - carriage_angle_;
        if (std::abs(angle_error) > allowable_error_) {
            if (angle_error > 0.0) {
                command_ = rmcs_msgs::DartMechanismCommand::DOWN;
                target_velocity_limit_ = carriage_down_velocity_;
            } else {
                command_ = rmcs_msgs::DartMechanismCommand::UP;
                target_velocity_limit_ = carriage_up_velocity_;
            }
        }

        apply_outputs();
    }

    ActionStatus update() override {
        if (!active_profile_.has_value()) {
            log_configuration_error();
            return fail(ActionFailureReason::CONFIGURATION_ERROR);
        }

        if (std::abs(carriage_angle_ - target_angle_) <= allowable_error_) {
            return ActionStatus::SUCCESS;
        }

        if (elapsed_ticks() >= timeout_ticks_) {
            return fail(ActionFailureReason::TIMEOUT);
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        carriage_command_ = rmcs_msgs::DartMechanismCommand::WAIT;
        carriage_target_velocity_ = 0.0;
        carriage_target_angle_ = std::numeric_limits<double>::quiet_NaN();
    }

private:
    void apply_outputs() {
        carriage_command_ = command_;
        carriage_target_velocity_ = target_velocity_limit_;
        carriage_target_angle_ = target_angle_;
    }

    void log_configuration_error() const {
        if (configuration_error_message_.empty() || runtime_context().logger == nullptr) {
            return;
        }

        RCLCPP_ERROR(
            *runtime_context().logger, "[TriggerCarriagePositionAimAction] %s",
            configuration_error_message_.c_str());
    }

    rmcs_msgs::DartMechanismCommand& carriage_command_;
    double& carriage_target_velocity_;
    double& carriage_target_angle_;
    const double& carriage_angle_;
    const double& carriage_origin_angle_;
    const VisionAimProfileProvider& profile_provider_;
    uint32_t fire_count_;
    double carriage_down_velocity_;
    double carriage_up_velocity_;
    double allowable_error_;
    uint64_t timeout_ticks_;

    std::optional<VisionAimRuntimeProfile> active_profile_;
    std::string configuration_error_message_;
    rmcs_msgs::DartMechanismCommand command_{rmcs_msgs::DartMechanismCommand::WAIT};
    double target_angle_{std::numeric_limits<double>::quiet_NaN()};
    double target_velocity_limit_{0.0};
};

} // namespace rmcs_dart_guidance::manager
