#pragma once

#include <cmath>
#include <limits>

#include <eigen3/Eigen/Dense>
#include <string>

#include "manager/core/runtime/action.hpp"
#include "rmcs_msgs/dart_mechanism_command.hpp"
#include "rmcs_msgs/dart_motor_exit_mode.hpp"
#include "rmcs_msgs/dart_servo_command.hpp"
#include "rmcs_msgs/switch.hpp"

namespace rmcs_dart_guidance::manager {

class ManualControlAction : public IAction {
public:
    ManualControlAction(
        std::string name,                                   //
        const rmcs_msgs::Switch& remote_left_switch,        //
        const rmcs_msgs::Switch& remote_right_switch,       //
        const rmcs_msgs::Switch& remote_rotary_knob_switch, //
        const Eigen::Vector2d& remote_left_joystic,         //
        const Eigen::Vector2d& remote_right_joystic,        //
        rmcs_msgs::DartMechanismCommand& belt_command,      //
        double& belt_target_velocity,                       //
        rmcs_msgs::ExitMode& belt_exit_mode,                //
        rmcs_msgs::DartServoCommand& trigger_command,       //
        double& carriage_target_velocity,                     //
        double& force_max_velocity_override,                //
        int32_t& force_error,                               //
        Eigen::Vector2d& angle_error_vector,                //
        double angle_max_error,                             //
        int32_t force_max_error,                            //
        double belt_max_velocity                            //
        )
        : IAction(std::move(name))
        , remote_left_switch_(remote_left_switch)
        , remote_right_switch_(remote_right_switch)
        , remote_rotary_knob_switch_(remote_rotary_knob_switch)
        , remote_left_joystic_(remote_left_joystic)
        , remote_right_joystic_(remote_right_joystic)
        , belt_command_output_interface_(belt_command)
        , belt_target_velocity_output_interface_(belt_target_velocity)
        , belt_exit_mode_output_interface_(belt_exit_mode)
        , trigger_command_output_interface_(trigger_command)
        , carriage_target_velocity_output_interface_(carriage_target_velocity)
        , force_max_velocity_override_output_interface_(force_max_velocity_override)
        , force_error_interface_(force_error)
        , angle_error_vector_output_interface_(angle_error_vector)
        , angle_max_error_(angle_max_error)
        , force_max_error_(force_max_error)
        , belt_max_velocity_(belt_max_velocity) {}

    void on_enter() override {
        belt_exit_mode_output_interface_ = rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY;
        belt_command_output_interface_ = rmcs_msgs::DartMechanismCommand::WAIT;
        belt_target_velocity_output_interface_ = 0.0;
        apply_trigger_command();
        carriage_target_velocity_output_interface_ = 0.0;
        force_max_velocity_override_output_interface_ = std::numeric_limits<double>::quiet_NaN();
        force_error_interface_ = 0;
        angle_error_vector_output_interface_ = Eigen::Vector2d::Zero();
    }

    ActionStatus update() override {
        if (remote_left_switch_ != rmcs_msgs::Switch::UP) {
            return ActionStatus::SUCCESS;
        }

        belt_exit_mode_output_interface_ = rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY;
        belt_command_output_interface_ = rmcs_msgs::DartMechanismCommand::WAIT;
        belt_target_velocity_output_interface_ = 0.0;
        carriage_target_velocity_output_interface_ = 0.0;
        force_max_velocity_override_output_interface_ = std::numeric_limits<double>::quiet_NaN();
        force_error_interface_ = 0;
        angle_error_vector_output_interface_ = Eigen::Vector2d::Zero();

        if (remote_right_switch_ == rmcs_msgs::Switch::UP) {
            Eigen::Vector2d angle_error_vector(
                remote_left_joystic_.y() * angle_max_error_,
                remote_right_joystic_.x() * angle_max_error_);
            angle_error_vector_output_interface_ = angle_error_vector;

        } else if (remote_right_switch_ == rmcs_msgs::Switch::MIDDLE) {
            apply_trigger_command();

            const double belt_velocity = remote_left_joystic_.x() * belt_max_velocity_;
            if (belt_velocity > 0.0) {
                belt_command_output_interface_ = rmcs_msgs::DartMechanismCommand::UP;
                belt_target_velocity_output_interface_ = std::abs(belt_velocity);
            } else if (belt_velocity < 0.0) {
                belt_command_output_interface_ = rmcs_msgs::DartMechanismCommand::DOWN;
                belt_target_velocity_output_interface_ = std::abs(belt_velocity);
            }

            const double raw_force_error = -remote_right_joystic_.x() * force_max_error_;
            force_error_interface_ = clamp_to_int32(std::lround(raw_force_error));
            carriage_target_velocity_output_interface_ = 0.0;
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        belt_command_output_interface_ = rmcs_msgs::DartMechanismCommand::WAIT;
        belt_target_velocity_output_interface_ = 0.0;
        belt_exit_mode_output_interface_ = rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY;
        carriage_target_velocity_output_interface_ = 0.0;
        force_max_velocity_override_output_interface_ = std::numeric_limits<double>::quiet_NaN();
        force_error_interface_ = 0;
        angle_error_vector_output_interface_ = Eigen::Vector2d::Zero();
    }

private:
    void apply_trigger_command() {
        switch (remote_rotary_knob_switch_) {
        case rmcs_msgs::Switch::UP:
            trigger_command_output_interface_ = rmcs_msgs::DartServoCommand::LOCK;
            break;
        case rmcs_msgs::Switch::DOWN:
            trigger_command_output_interface_ = rmcs_msgs::DartServoCommand::FREE;
            break;
        case rmcs_msgs::Switch::MIDDLE:
        case rmcs_msgs::Switch::UNKNOWN: break;
        }
    }

    static int32_t clamp_to_int32(const long value) {
        if (value > std::numeric_limits<int32_t>::max()) {
            return std::numeric_limits<int32_t>::max();
        }
        if (value < std::numeric_limits<int32_t>::min()) {
            return std::numeric_limits<int32_t>::min();
        }
        return static_cast<int32_t>(value);
    }

    const rmcs_msgs::Switch& remote_left_switch_;
    const rmcs_msgs::Switch& remote_right_switch_;
    const rmcs_msgs::Switch& remote_rotary_knob_switch_;
    const Eigen::Vector2d& remote_left_joystic_;
    const Eigen::Vector2d& remote_right_joystic_;

    rmcs_msgs::DartMechanismCommand& belt_command_output_interface_;
    double& belt_target_velocity_output_interface_;
    rmcs_msgs::ExitMode& belt_exit_mode_output_interface_;
    rmcs_msgs::DartServoCommand& trigger_command_output_interface_;
    double& carriage_target_velocity_output_interface_;
    double& force_max_velocity_override_output_interface_;
    int32_t& force_error_interface_;
    Eigen::Vector2d& angle_error_vector_output_interface_;

    double angle_max_error_;
    int32_t force_max_error_;
    double belt_max_velocity_;
};
} // namespace rmcs_dart_guidance::manager
