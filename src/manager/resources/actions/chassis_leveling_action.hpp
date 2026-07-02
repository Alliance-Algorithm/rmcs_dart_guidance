#pragma once

#include <cmath>
#include <string>
#include <utility>

#include <eigen3/Eigen/Dense>

#include "manager/core/runtime/action.hpp"
#include "rmcs_msgs/chassis_leveling_phase.hpp"

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// RollLevelingAction
//   滚动调平动作：根据IMU姿态信息和底盘调平电机速度设定值进行调平控制。
// ─────────────────────────────────────────────────────────────────────────────
class RollLevelingAction : public IAction {
public:
    RollLevelingAction(
        std::string name,                                        //
        rmcs_msgs::ChassisLevelingPhase& chassis_leveling_phase, //
        const double& leveling_front_left_velocity,              //
        const double& leveling_rear_left_velocity,               //
        const double& leveling_front_left_torque,                //
        const double& leveling_rear_left_torque,                 //
        const double& imu_roll                                   //
        )
        : IAction(std::move(name))
        , chassis_leveling_phase_output_interface_(chassis_leveling_phase)
        , leveling_front_left_velocity_(leveling_front_left_velocity)
        , leveling_rear_left_velocity_(leveling_rear_left_velocity)
        , leveling_front_left_torque_(leveling_front_left_torque)
        , leveling_rear_left_torque_(leveling_rear_left_torque)
        , imu_roll_(imu_roll) {}

    void on_enter() override {
        stall_counter_ = 0;
        chassis_leveling_phase_output_interface_ = rmcs_msgs::ChassisLevelingPhase::ROLL;
        max_error_ = 0.02;
    }

    ActionStatus update() override {
        if (elapsed_ticks() > 200) {
            if (std::abs(leveling_front_left_velocity_) <= 0.05
                && std::abs(leveling_rear_left_velocity_) <= 0.05
                && leveling_front_left_torque_ >= 0.5 && leveling_rear_left_torque_ >= 0.5) {
                ++stall_counter_;
                if (stall_counter_ >= 500) {
                    return fail(ActionFailureReason::STALL);
                }
            } else {
                stall_counter_ = 0;
            }
        }

        if (std::abs(imu_roll_) <= max_error_) {
            return ActionStatus::SUCCESS;
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        stall_counter_ = 0;
        chassis_leveling_phase_output_interface_ = rmcs_msgs::ChassisLevelingPhase::WAIT;
    }

private:
    rmcs_msgs::ChassisLevelingPhase& chassis_leveling_phase_output_interface_;
    const double& leveling_front_left_velocity_;
    const double& leveling_rear_left_velocity_;
    const double& leveling_front_left_torque_;
    const double& leveling_rear_left_torque_;
    const double& imu_roll_;
    double max_error_;
    int stall_counter_;
};

class PitchLevelingAction : public IAction {
public:
    PitchLevelingAction(
        std::string name,                                        //
        rmcs_msgs::ChassisLevelingPhase& chassis_leveling_phase, //
        const double& leveling_front_left_velocity,              //
        const double& leveling_front_right_velocity,             //
        const double& leveling_front_left_torque,                //
        const double& leveling_front_right_torque,               //
        const double& imu_pitch                                  //
        )
        : IAction(std::move(name))
        , chassis_leveling_phase_output_interface_(chassis_leveling_phase)
        , leveling_front_left_velocity_(leveling_front_left_velocity)
        , leveling_front_right_velocity_(leveling_front_right_velocity)
        , leveling_front_left_torque_(leveling_front_left_torque)
        , leveling_front_right_torque_(leveling_front_right_torque)
        , imu_pitch_(imu_pitch) {}

    void on_enter() override {
        stall_counter_ = 0;
        chassis_leveling_phase_output_interface_ = rmcs_msgs::ChassisLevelingPhase::PITCH;
        max_error_ = 0.02;
    }

    ActionStatus update() override {
        if (elapsed_ticks() > 200) {
            if (std::abs(leveling_front_left_velocity_) <= 0.05
                && std::abs(leveling_front_right_velocity_) <= 0.05
                && leveling_front_left_torque_ >= 0.5 && leveling_front_right_torque_ >= 0.5) {
                ++stall_counter_;
                if (stall_counter_ >= 500) {
                    return fail(ActionFailureReason::STALL);
                }
            } else {
                stall_counter_ = 0;
            }
        }

        if (std::abs(imu_pitch_) <= max_error_) {
            return ActionStatus::SUCCESS;
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        stall_counter_ = 0;
        chassis_leveling_phase_output_interface_ = rmcs_msgs::ChassisLevelingPhase::WAIT;
    }

private:
    rmcs_msgs::ChassisLevelingPhase& chassis_leveling_phase_output_interface_;
    const double& leveling_front_left_velocity_;
    const double& leveling_front_right_velocity_;
    const double& leveling_front_left_torque_;
    const double& leveling_front_right_torque_;
    const double& imu_pitch_;
    double max_error_;
    int stall_counter_;
};
} // namespace rmcs_dart_guidance::manager
