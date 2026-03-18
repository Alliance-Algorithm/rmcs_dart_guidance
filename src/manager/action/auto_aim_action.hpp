#pragma once

#include "action.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <utility>

#include <eigen3/Eigen/Dense>
#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace rmcs_dart_guidance::manager {

struct AutoAimParams {
    Eigen::Vector2d desired_target_px = Eigen::Vector2d::Zero();
    Eigen::Vector2d deadband_px = Eigen::Vector2d::Constant(3.0);
    Eigen::Vector2d ready_exit_deadband_px = Eigen::Vector2d::Constant(5.0);
    Eigen::Vector2d accept_deadband_px = Eigen::Vector2d::Constant(8.0);
    double yaw_gain = 0.0;
    double pitch_gain = 0.0;
    uint64_t ready_confirm_ticks = 5;
    uint64_t timeout_ticks = 3000;
    double min_transform_rate = 0.0;
    double max_transform_rate = 500.0;
};

// AutoAimAction:
//   - 在上膛前执行视觉辅助预瞄
//   - 严格对准进入 ready，误差在更宽的 accept_deadband 内也可直接放行
//   - tracking 丢失不失败，等待重新跟踪或超时
//   - 超时始终返回 SUCCESS，后续流程继续执行
class AutoAimAction : public IAction {
public:
    AutoAimAction(
        Eigen::Vector2d& yaw_pitch_control_velocity, bool& aim_ready, Eigen::Vector2d& aim_error_px,
        Eigen::Vector2d& desired_target_px_output, const cv::Point2i& current_target_position,
        const bool& tracking, const rclcpp::Logger& logger, AutoAimParams params)
        : IAction("auto_aim")
        , yaw_pitch_control_velocity_(yaw_pitch_control_velocity)
        , aim_ready_(aim_ready)
        , aim_error_px_(aim_error_px)
        , desired_target_px_output_(desired_target_px_output)
        , current_target_position_(current_target_position)
        , tracking_(tracking)
        , logger_(logger)
        , params_(std::move(params)) {}

    void on_enter() override {
        strict_ready_ticks_ = 0;
        acceptable_ticks_ = 0;
        completed_successfully_ = false;
        last_valid_error_px_ = Eigen::Vector2d::Zero();
        last_tracking_valid_ = false;
        has_seen_valid_target_ = false;
        aim_ready_ = false;
        aim_error_px_ = Eigen::Vector2d::Zero();
        desired_target_px_output_ = params_.desired_target_px;
        yaw_pitch_control_velocity_ = Eigen::Vector2d::Zero();
        last_log_time_ = std::chrono::steady_clock::now();
    }

    ActionStatus update() override {
        desired_target_px_output_ = params_.desired_target_px;

        if (elapsed_ticks() >= params_.timeout_ticks) {
            completed_successfully_ = true;
            aim_ready_ = false;
            aim_error_px_ = has_seen_valid_target_ ? last_valid_error_px_ : Eigen::Vector2d::Zero();
            yaw_pitch_control_velocity_ = Eigen::Vector2d::Zero();
            log_timeout();
            return ActionStatus::SUCCESS;
        }

        const bool tracking_valid =
            tracking_ && current_target_position_.x >= 0 && current_target_position_.y >= 0;
        last_tracking_valid_ = tracking_valid;
        if (!tracking_valid) {
            strict_ready_ticks_ = 0;
            acceptable_ticks_ = 0;
            aim_ready_ = false;
            aim_error_px_ = Eigen::Vector2d::Zero();
            yaw_pitch_control_velocity_ = Eigen::Vector2d::Zero();
            log_status(current_target_position_, aim_error_px_);
            return ActionStatus::RUNNING;
        }

        const Eigen::Vector2d current_target_px(
            static_cast<double>(current_target_position_.x),
            static_cast<double>(current_target_position_.y));
        const Eigen::Vector2d error_px = params_.desired_target_px - current_target_px;
        has_seen_valid_target_ = true;
        last_valid_error_px_ = error_px;
        aim_error_px_ = error_px;

        const Eigen::Vector2d ready_exit_deadband_px =
            params_.ready_exit_deadband_px.cwiseMax(params_.deadband_px);
        const Eigen::Vector2d accept_deadband_px =
            params_.accept_deadband_px.cwiseMax(ready_exit_deadband_px);

        const bool within_deadband =
            std::abs(error_px.x()) <= params_.deadband_px.x()
            && std::abs(error_px.y()) <= params_.deadband_px.y();
        const bool within_ready_hold_deadband =
            std::abs(error_px.x()) <= ready_exit_deadband_px.x()
            && std::abs(error_px.y()) <= ready_exit_deadband_px.y();
        const bool within_accept_deadband =
            std::abs(error_px.x()) <= accept_deadband_px.x()
            && std::abs(error_px.y()) <= accept_deadband_px.y();

        if (aim_ready_ && within_ready_hold_deadband) {
            yaw_pitch_control_velocity_ = Eigen::Vector2d::Zero();
            completed_successfully_ = true;
            log_status(current_target_position_, error_px);
            return ActionStatus::SUCCESS;
        }

        if (within_deadband) {
            yaw_pitch_control_velocity_ = Eigen::Vector2d::Zero();
            acceptable_ticks_ = 0;
            ++strict_ready_ticks_;
            aim_ready_ = strict_ready_ticks_ >= params_.ready_confirm_ticks;
            log_status(current_target_position_, error_px);
            if (aim_ready_) {
                completed_successfully_ = true;
                return ActionStatus::SUCCESS;
            }
            return ActionStatus::RUNNING;
        }

        if (within_accept_deadband) {
            strict_ready_ticks_ = 0;
            acceptable_ticks_++;
            aim_ready_ = false;
            yaw_pitch_control_velocity_ = Eigen::Vector2d::Zero();
            log_status(current_target_position_, error_px);
            if (acceptable_ticks_ >= params_.ready_confirm_ticks) {
                completed_successfully_ = true;
                return ActionStatus::SUCCESS;
            }
            return ActionStatus::RUNNING;
        }

        strict_ready_ticks_ = 0;
        acceptable_ticks_ = 0;
        aim_ready_ = false;
        yaw_pitch_control_velocity_.x() = compute_axis_velocity(error_px.x(), params_.yaw_gain);
        yaw_pitch_control_velocity_.y() = compute_axis_velocity(error_px.y(), params_.pitch_gain);
        log_status(current_target_position_, error_px);
        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        yaw_pitch_control_velocity_ = Eigen::Vector2d::Zero();
        if (completed_successfully_) {
            return;
        }

        aim_ready_ = false;
        aim_error_px_ = Eigen::Vector2d::Zero();
    }

private:
    void log_status(const cv::Point2i& current_target_position, const Eigen::Vector2d& error_px) {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_log_time_ < std::chrono::seconds(1)) {
            return;
        }

        last_log_time_ = now;
        RCLCPP_INFO(
            logger_,
            "[AutoAimAction] desired=(%.1f, %.1f) current=(%d, %d) error=(%.1f, %.1f)",
            params_.desired_target_px.x(), params_.desired_target_px.y(), current_target_position.x,
            current_target_position.y, error_px.x(), error_px.y());
    }

    void log_timeout() const {
        RCLCPP_WARN(
            logger_,
            "[AutoAimAction] timeout: tracking=%s last_error=(%.1f, %.1f)",
            last_tracking_valid_ ? "true" : "false", aim_error_px_.x(), aim_error_px_.y());
    }

    double compute_axis_velocity(double error_px, double gain) const {
        if (std::abs(error_px) <= 0.0 || std::abs(gain) <= 0.0) {
            return 0.0;
        }

        const double velocity = error_px * gain;
        const double bounded_velocity = std::clamp(
            velocity, -params_.max_transform_rate, params_.max_transform_rate);
        const double bounded_magnitude = std::abs(bounded_velocity);
        if (bounded_magnitude <= 0.0) {
            return 0.0;
        }

        const double final_magnitude = std::clamp(
            std::max(bounded_magnitude, params_.min_transform_rate),
            0.0, params_.max_transform_rate);
        return std::copysign(final_magnitude, bounded_velocity);
    }

    Eigen::Vector2d& yaw_pitch_control_velocity_;
    bool& aim_ready_;
    Eigen::Vector2d& aim_error_px_;
    Eigen::Vector2d& desired_target_px_output_;

    const cv::Point2i& current_target_position_;
    const bool& tracking_;
    rclcpp::Logger logger_;
    AutoAimParams params_;

    uint64_t strict_ready_ticks_{0};
    uint64_t acceptable_ticks_{0};
    bool completed_successfully_{false};
    Eigen::Vector2d last_valid_error_px_{Eigen::Vector2d::Zero()};
    bool last_tracking_valid_{false};
    bool has_seen_valid_target_{false};
    std::chrono::steady_clock::time_point last_log_time_{};
};

} // namespace rmcs_dart_guidance::manager
