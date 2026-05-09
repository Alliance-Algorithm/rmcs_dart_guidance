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
        rmcs_msgs::DartMechanismCommand& lift_command,      //
        double& lift_target_velocity,                       //
        rmcs_msgs::ExitMode& lift_exit_mode,                //
        rmcs_msgs::DartServoCommand& trigger_command,       //
        int32_t& force_error,                               //
        Eigen::Vector2d& angle_error_vector,                //
        double angle_max_error,                             //
        int32_t force_max_error,                            //
        double belt_max_velocity,                           //
        double lift_target_velocity_setting                 //
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
        , lift_command_output_interface_(lift_command)
        , lift_target_velocity_output_interface_(lift_target_velocity)
        , lift_exit_mode_output_interface_(lift_exit_mode)
        , trigger_command_output_interface_(trigger_command)
        , force_error_interface_(force_error)
        , angle_error_vector_output_interface_(angle_error_vector)
        , angle_max_error_(angle_max_error)
        , force_max_error_(force_max_error)
        , belt_max_velocity_(belt_max_velocity)
        , lift_target_velocity_setting_(lift_target_velocity_setting) {}

    void on_enter() override {
        reset_belt_output();
        reset_lift_output();
        trigger_command_output_interface_ = rmcs_msgs::DartServoCommand::WAIT;
        force_error_interface_ = 0;
        angle_error_vector_output_interface_ = Eigen::Vector2d::Zero();
    }

    ActionStatus update() override {
        if (remote_left_switch_ != rmcs_msgs::Switch::UP) {
            return ActionStatus::SUCCESS;
        }

        reset_belt_output();
        reset_lift_output();
        trigger_command_output_interface_ = rmcs_msgs::DartServoCommand::WAIT;
        force_error_interface_ = 0;
        angle_error_vector_output_interface_ = Eigen::Vector2d::Zero();

        if (remote_right_switch_ == rmcs_msgs::Switch::UP) {
            Eigen::Vector2d angle_error_vector(
                remote_left_joystic_.y() * angle_max_error_,
                remote_right_joystic_.x() * angle_max_error_);
            angle_error_vector_output_interface_ = angle_error_vector;
            apply_lift_command();

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
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        reset_belt_output();
        reset_lift_output();
        force_error_interface_ = 0;
        angle_error_vector_output_interface_ = Eigen::Vector2d::Zero();
    }

private:
    void reset_belt_output() {
        belt_command_output_interface_ = rmcs_msgs::DartMechanismCommand::WAIT;
        belt_target_velocity_output_interface_ = 0.0;
        belt_exit_mode_output_interface_ = rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY;
    }

    void reset_lift_output() {
        lift_command_output_interface_ = rmcs_msgs::DartMechanismCommand::WAIT;
        lift_target_velocity_output_interface_ = 0.0;
        lift_exit_mode_output_interface_ = rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY;
    }

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

    void apply_lift_command() {
        switch (remote_rotary_knob_switch_) {
        case rmcs_msgs::Switch::UP:
            lift_command_output_interface_ = rmcs_msgs::DartMechanismCommand::UP;
            lift_target_velocity_output_interface_ = lift_target_velocity_setting_;
            break;
        case rmcs_msgs::Switch::DOWN:
            lift_command_output_interface_ = rmcs_msgs::DartMechanismCommand::DOWN;
            lift_target_velocity_output_interface_ = lift_target_velocity_setting_;
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
    rmcs_msgs::DartMechanismCommand& lift_command_output_interface_;
    double& lift_target_velocity_output_interface_;
    rmcs_msgs::ExitMode& lift_exit_mode_output_interface_;
    rmcs_msgs::DartServoCommand& trigger_command_output_interface_;
    int32_t& force_error_interface_;
    Eigen::Vector2d& angle_error_vector_output_interface_;

    double angle_max_error_;
    int32_t force_max_error_;
    double belt_max_velocity_;
    double lift_target_velocity_setting_;
};
} // namespace rmcs_dart_guidance::manager
