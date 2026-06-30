#pragma once

#include "manager/core/runtime/action.hpp"
#include "rmcs_msgs/chassis_leveling_phase.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace rmcs_dart_guidance::manager {

struct MotorFeedbackView {
    const double& velocity;
    const double& torque;
};

struct LevelingWheelFeedback {
    MotorFeedbackView front_left;
    MotorFeedbackView front_right;
    MotorFeedbackView rear_left;
    MotorFeedbackView rear_right;
};

struct LevelingMonitorParameters {
    double stall_velocity_threshold;
    double stall_torque_threshold;
    uint64_t stall_confirm_ticks;
    uint64_t min_run_ticks;
    double angle_allowable_error;
    uint64_t angle_confirm_ticks;
    uint64_t timeout_ticks;
};

namespace detail {

inline double max_abs(const std::initializer_list<double> values) {
    double result = 0.0;
    for (const double value : values) {
        result = std::max(result, std::abs(value));
    }
    return result;
}

inline double min_abs(const std::initializer_list<double> values) {
    double result = std::numeric_limits<double>::infinity();
    for (const double value : values) {
        result = std::min(result, std::abs(value));
    }
    return std::isfinite(result) ? result : 0.0;
}

inline bool all_finite(const std::initializer_list<double> values) {
    return std::ranges::all_of(values, [](double value) { return std::isfinite(value); });
}

} // namespace detail

class LevelingMonitorActionBase : public IAction {
public:
    LevelingMonitorActionBase(
        std::string name, rmcs_msgs::ChassisLevelingPhase& chassis_leveling_phase_interface,
        rmcs_msgs::ChassisLevelingPhase active_phase, LevelingMonitorParameters parameters)
        : IAction(std::move(name))
        , chassis_leveling_phase_output_interface_(chassis_leveling_phase_interface)
        , active_phase_(active_phase)
        , parameters_(parameters) {}

    void on_enter() override {
        chassis_leveling_phase_output_interface_ = active_phase_;
        stall_counter_ = 0;
        confirm_counter_ = 0;
    }

    ActionStatus update() override {
        if (!inputs_are_valid()) {
            return fail(ActionFailureReason::INVALID_INPUT);
        }

        if (elapsed_ticks() >= parameters_.timeout_ticks) {
            return fail(ActionFailureReason::TIMEOUT);
        }

        if (elapsed_ticks() <= parameters_.min_run_ticks) {
            stall_counter_ = 0;
            confirm_counter_ = 0;
            return ActionStatus::RUNNING;
        }

        if (const auto error_abs = current_error_abs()) {
            if (*error_abs <= parameters_.angle_allowable_error) {
                ++confirm_counter_;
                stall_counter_ = 0;
                if (confirm_counter_ >= parameters_.angle_confirm_ticks) {
                    return ActionStatus::SUCCESS;
                }
                return ActionStatus::RUNNING;
            }
            confirm_counter_ = 0;
        }

        if (monitored_velocity_abs() <= parameters_.stall_velocity_threshold
            && monitored_torque_abs() >= parameters_.stall_torque_threshold) {
            ++stall_counter_;
            if (stall_counter_ >= parameters_.stall_confirm_ticks) {
                return on_stall_confirmed();
            }
        } else {
            stall_counter_ = 0;
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        chassis_leveling_phase_output_interface_ = rmcs_msgs::ChassisLevelingPhase::IDLE;
    }

protected:
    virtual bool inputs_are_valid() const = 0;
    virtual double monitored_velocity_abs() const = 0;
    virtual double monitored_torque_abs() const = 0;
    virtual std::optional<double> current_error_abs() const { return std::nullopt; }

    virtual ActionStatus on_stall_confirmed() { return fail(ActionFailureReason::STALL); }

private:
    rmcs_msgs::ChassisLevelingPhase& chassis_leveling_phase_output_interface_;
    rmcs_msgs::ChassisLevelingPhase active_phase_;
    LevelingMonitorParameters parameters_;
    uint64_t stall_counter_{0};
    uint64_t confirm_counter_{0};
};

class LegacyRollLevelingAction : public LevelingMonitorActionBase {
public:
    LegacyRollLevelingAction(
        std::string name, rmcs_msgs::ChassisLevelingPhase& chassis_leveling_phase_interface,
        const double& imu_roll_angle_interface, LevelingWheelFeedback wheel_feedback,
        LevelingMonitorParameters parameters)
        : LevelingMonitorActionBase(
              std::move(name), chassis_leveling_phase_interface,
              rmcs_msgs::ChassisLevelingPhase::LEGACY_ROLL, parameters)
        , imu_roll_angle_input_interface_(imu_roll_angle_interface)
        , wheel_feedback_(wheel_feedback) {}

protected:
    bool inputs_are_valid() const override {
        return detail::all_finite(
            {imu_roll_angle_input_interface_, wheel_feedback_.front_left.velocity,
             wheel_feedback_.front_left.torque, wheel_feedback_.rear_left.velocity,
             wheel_feedback_.rear_left.torque});
    }

    double monitored_velocity_abs() const override {
        return detail::min_abs(
            {wheel_feedback_.front_left.velocity, wheel_feedback_.rear_left.velocity});
    }

    double monitored_torque_abs() const override {
        return detail::max_abs(
            {wheel_feedback_.front_left.torque, wheel_feedback_.rear_left.torque});
    }

    std::optional<double> current_error_abs() const override {
        return std::abs(imu_roll_angle_input_interface_);
    }

private:
    const double& imu_roll_angle_input_interface_;
    LevelingWheelFeedback wheel_feedback_;
};

class LegacyPitchLevelingAction : public LevelingMonitorActionBase {
public:
    LegacyPitchLevelingAction(
        std::string name, rmcs_msgs::ChassisLevelingPhase& chassis_leveling_phase_interface,
        const double& imu_pitch_angle_interface, LevelingWheelFeedback wheel_feedback,
        LevelingMonitorParameters parameters)
        : LevelingMonitorActionBase(
              std::move(name), chassis_leveling_phase_interface,
              rmcs_msgs::ChassisLevelingPhase::LEGACY_PITCH, parameters)
        , imu_pitch_angle_input_interface_(imu_pitch_angle_interface)
        , wheel_feedback_(wheel_feedback) {}

protected:
    bool inputs_are_valid() const override {
        return detail::all_finite(
            {imu_pitch_angle_input_interface_, wheel_feedback_.front_left.velocity,
             wheel_feedback_.front_left.torque, wheel_feedback_.front_right.velocity,
             wheel_feedback_.front_right.torque});
    }

    double monitored_velocity_abs() const override {
        return detail::min_abs(
            {wheel_feedback_.front_left.velocity, wheel_feedback_.front_right.velocity});
    }

    double monitored_torque_abs() const override {
        return detail::max_abs(
            {wheel_feedback_.front_left.torque, wheel_feedback_.front_right.torque});
    }

    std::optional<double> current_error_abs() const override {
        return std::abs(imu_pitch_angle_input_interface_);
    }

private:
    const double& imu_pitch_angle_input_interface_;
    LevelingWheelFeedback wheel_feedback_;
};

class StageContactLevelingAction : public LevelingMonitorActionBase {
public:
    StageContactLevelingAction(
        std::string name, rmcs_msgs::ChassisLevelingPhase& chassis_leveling_phase_interface,
        LevelingWheelFeedback wheel_feedback, LevelingMonitorParameters parameters)
        : LevelingMonitorActionBase(
              std::move(name), chassis_leveling_phase_interface,
              rmcs_msgs::ChassisLevelingPhase::STAGE_CONTACT, parameters)
        , wheel_feedback_(wheel_feedback) {}

protected:
    bool inputs_are_valid() const override {
        return detail::all_finite(
            {wheel_feedback_.front_left.velocity, wheel_feedback_.front_left.torque,
             wheel_feedback_.front_right.velocity, wheel_feedback_.front_right.torque,
             wheel_feedback_.rear_left.velocity, wheel_feedback_.rear_left.torque,
             wheel_feedback_.rear_right.velocity, wheel_feedback_.rear_right.torque});
    }

    double monitored_velocity_abs() const override {
        return detail::max_abs(
            {wheel_feedback_.front_left.velocity, wheel_feedback_.front_right.velocity,
             wheel_feedback_.rear_left.velocity, wheel_feedback_.rear_right.velocity});
    }

    double monitored_torque_abs() const override {
        return detail::max_abs(
            {wheel_feedback_.front_left.torque, wheel_feedback_.front_right.torque,
             wheel_feedback_.rear_left.torque, wheel_feedback_.rear_right.torque});
    }

    ActionStatus on_stall_confirmed() override { return ActionStatus::SUCCESS; }

private:
    LevelingWheelFeedback wheel_feedback_;
};

class StageRollLevelingAction : public LevelingMonitorActionBase {
public:
    StageRollLevelingAction(
        std::string name, rmcs_msgs::ChassisLevelingPhase& chassis_leveling_phase_interface,
        const double& imu_roll_angle_interface, LevelingWheelFeedback wheel_feedback,
        LevelingMonitorParameters parameters)
        : LevelingMonitorActionBase(
              std::move(name), chassis_leveling_phase_interface,
              rmcs_msgs::ChassisLevelingPhase::STAGE_ROLL, parameters)
        , imu_roll_angle_input_interface_(imu_roll_angle_interface)
        , wheel_feedback_(wheel_feedback) {}

protected:
    bool inputs_are_valid() const override {
        return detail::all_finite(
            {imu_roll_angle_input_interface_, wheel_feedback_.front_left.velocity,
             wheel_feedback_.front_left.torque, wheel_feedback_.front_right.velocity,
             wheel_feedback_.front_right.torque, wheel_feedback_.rear_left.velocity,
             wheel_feedback_.rear_left.torque, wheel_feedback_.rear_right.velocity,
             wheel_feedback_.rear_right.torque});
    }

    double monitored_velocity_abs() const override {
        if (imu_roll_angle_input_interface_ < 0.0) {
            return detail::max_abs(
                {wheel_feedback_.front_left.velocity, wheel_feedback_.rear_left.velocity});
        }
        if (imu_roll_angle_input_interface_ > 0.0) {
            return detail::max_abs(
                {wheel_feedback_.front_right.velocity, wheel_feedback_.rear_right.velocity});
        }
        return 0.0;
    }

    double monitored_torque_abs() const override {
        if (imu_roll_angle_input_interface_ < 0.0) {
            return detail::max_abs(
                {wheel_feedback_.front_left.torque, wheel_feedback_.rear_left.torque});
        }
        if (imu_roll_angle_input_interface_ > 0.0) {
            return detail::max_abs(
                {wheel_feedback_.front_right.torque, wheel_feedback_.rear_right.torque});
        }
        return 0.0;
    }

    std::optional<double> current_error_abs() const override {
        return std::abs(imu_roll_angle_input_interface_);
    }

private:
    const double& imu_roll_angle_input_interface_;
    LevelingWheelFeedback wheel_feedback_;
};

class StagePitchLevelingAction : public LevelingMonitorActionBase {
public:
    StagePitchLevelingAction(
        std::string name, rmcs_msgs::ChassisLevelingPhase& chassis_leveling_phase_interface,
        const double& imu_pitch_angle_interface, LevelingWheelFeedback wheel_feedback,
        LevelingMonitorParameters parameters)
        : LevelingMonitorActionBase(
              std::move(name), chassis_leveling_phase_interface,
              rmcs_msgs::ChassisLevelingPhase::STAGE_PITCH, parameters)
        , imu_pitch_angle_input_interface_(imu_pitch_angle_interface)
        , wheel_feedback_(wheel_feedback) {}

protected:
    bool inputs_are_valid() const override {
        return detail::all_finite(
            {imu_pitch_angle_input_interface_, wheel_feedback_.front_left.velocity,
             wheel_feedback_.front_left.torque, wheel_feedback_.front_right.velocity,
             wheel_feedback_.front_right.torque, wheel_feedback_.rear_left.velocity,
             wheel_feedback_.rear_left.torque, wheel_feedback_.rear_right.velocity,
             wheel_feedback_.rear_right.torque});
    }

    double monitored_velocity_abs() const override {
        if (imu_pitch_angle_input_interface_ < 0.0) {
            return detail::max_abs(
                {wheel_feedback_.front_left.velocity, wheel_feedback_.front_right.velocity});
        }
        if (imu_pitch_angle_input_interface_ > 0.0) {
            return detail::max_abs(
                {wheel_feedback_.rear_left.velocity, wheel_feedback_.rear_right.velocity});
        }
        return 0.0;
    }

    double monitored_torque_abs() const override {
        if (imu_pitch_angle_input_interface_ < 0.0) {
            return detail::max_abs(
                {wheel_feedback_.front_left.torque, wheel_feedback_.front_right.torque});
        }
        if (imu_pitch_angle_input_interface_ > 0.0) {
            return detail::max_abs(
                {wheel_feedback_.rear_left.torque, wheel_feedback_.rear_right.torque});
        }
        return 0.0;
    }

    std::optional<double> current_error_abs() const override {
        return std::abs(imu_pitch_angle_input_interface_);
    }

private:
    const double& imu_pitch_angle_input_interface_;
    LevelingWheelFeedback wheel_feedback_;
};

} // namespace rmcs_dart_guidance::manager
