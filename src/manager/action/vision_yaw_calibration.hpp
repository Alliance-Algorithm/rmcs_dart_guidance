#pragma once

#include "action.hpp"

#include <eigen3/Eigen/Dense>
#include <opencv2/core/types.hpp>

namespace rmcs_dart_guidance::manager {

class VisionYawCalibration : public IAction {
public:
    VisionYawCalibration(
        Eigen::Vector2d& yaw_pitch_angle, const Eigen::Vector2d& yaw_pitch_distance,
        double yaw_angle_mapping_rate)
        : IAction("vision_yaw_calibration")
        , yaw_control_angle_(yaw_pitch_angle.x())
        , yaw_distance_(yaw_pitch_distance.x())
        , yaw_rate_(yaw_angle_mapping_rate) {}

    void on_enter() override {}

    ActionStatus update() override {
        if (elapsed_ticks() > 5000) {
            return ActionStatus::SUCCESS;
        }
        yaw_control_angle_ = yaw_distance_ * yaw_rate_;
        return ActionStatus::RUNNING;
    }

private:
    double& yaw_control_angle_;
    double yaw_distance_;
    double yaw_rate_;
};

} // namespace rmcs_dart_guidance::manager
