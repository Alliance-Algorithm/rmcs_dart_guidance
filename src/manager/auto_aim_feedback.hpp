#pragma once

#include <stdexcept>

#include <eigen3/Eigen/Dense>

namespace rmcs_dart_guidance::manager {

class AutoAimFeedback {
public:
    void bind(
        Eigen::Vector2d& yaw_pitch_control_velocity, bool& aim_ready,
        Eigen::Vector2d& aim_error_px, Eigen::Vector2d& desired_target_px) {
        yaw_pitch_control_velocity_ = &yaw_pitch_control_velocity;
        aim_ready_ = &aim_ready;
        aim_error_px_ = &aim_error_px;
        desired_target_px_ = &desired_target_px;
    }

    void reset(const Eigen::Vector2d& desired_target_px) {
        ensure_bound();
        *yaw_pitch_control_velocity_ = Eigen::Vector2d::Zero();
        *aim_ready_ = false;
        *aim_error_px_ = Eigen::Vector2d::Zero();
        *desired_target_px_ = desired_target_px;
    }

    void set_desired_target_px(const Eigen::Vector2d& desired_target_px) {
        ensure_bound();
        *desired_target_px_ = desired_target_px;
    }

    Eigen::Vector2d& yaw_pitch_control_velocity() {
        ensure_bound();
        return *yaw_pitch_control_velocity_;
    }

    bool& aim_ready() {
        ensure_bound();
        return *aim_ready_;
    }

    Eigen::Vector2d& aim_error_px() {
        ensure_bound();
        return *aim_error_px_;
    }

    Eigen::Vector2d& desired_target_px() {
        ensure_bound();
        return *desired_target_px_;
    }

private:
    void ensure_bound() const {
        if (
            yaw_pitch_control_velocity_ == nullptr || aim_ready_ == nullptr
            || aim_error_px_ == nullptr || desired_target_px_ == nullptr) {
            throw std::runtime_error("AutoAimFeedback is not bound");
        }
    }

    Eigen::Vector2d* yaw_pitch_control_velocity_{nullptr};
    bool* aim_ready_{nullptr};
    Eigen::Vector2d* aim_error_px_{nullptr};
    Eigen::Vector2d* desired_target_px_{nullptr};
};

} // namespace rmcs_dart_guidance::manager
