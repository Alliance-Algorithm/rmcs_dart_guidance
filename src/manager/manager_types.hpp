#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <opencv2/core/types.hpp>

#include "Eigen/src/Core/Matrix.h"
#include "manager/core/runtime/action.hpp"
#include "rmcs_msgs/dart_mechanism_command.hpp"
#include "rmcs_msgs/dart_motor_exit_mode.hpp"
#include "rmcs_msgs/dart_servo_command.hpp"
#include "rmcs_msgs/switch.hpp"

namespace rmcs_dart_guidance::manager {

enum class ManagerLifecycleState : uint8_t {
    IDLE = 0,
    RUNNING = 1,
    ERROR = 2,
};

inline const char* to_string(ManagerLifecycleState state) {
    switch (state) {
    case ManagerLifecycleState::IDLE: return "IDLE";
    case ManagerLifecycleState::RUNNING: return "RUNNING";
    case ManagerLifecycleState::ERROR: return "ERROR";
    }
    return "UNKNOWN";
}

struct ManagerInputContext {
    // belt
    const double& belt_left_angle;
    const double& belt_left_velocity;
    const double& belt_left_torque;
    const double& belt_right_angle;
    const double& belt_right_velocity;
    const double& belt_right_torque;

    // lift
    const double& lift_left_velocity;
    const double& lift_left_torque;
    const double& lift_right_velocity;
    const double& lift_right_torque;

    // carriage
    const double& carriage_angle;
    const double& carriage_velocity;
    const double& carriage_torque;

    // trigger

    // limit servo

    // yaw pitch force
    const int32_t& force_sensor_ch1;
    const int32_t& force_sensor_ch2;

    // vision
    const cv::Point2i& current_target;
    const bool& tracking;
    const uint64_t& target_seq;

    // remote control
    const rmcs_msgs::Switch& remote_left_switch;
    const rmcs_msgs::Switch& remote_right_switch;
    const rmcs_msgs::Switch& remote_rotary_knob_switch;
    const Eigen::Vector2d& remote_left_joystick;
    const Eigen::Vector2d& remote_right_joystick;
};

struct ManagerOutputContext {
    // belt
    rmcs_msgs::DartMechanismCommand& belt_command;
    double& belt_target_velocity;
    rmcs_msgs::ExitMode& belt_exit_mode;
    double& belt_max_torque_override;

    // lift
    rmcs_msgs::DartMechanismCommand& lifting_command;
    double& lift_target_velocity;
    rmcs_msgs::ExitMode& lift_exit_mode;

    // trigger
    rmcs_msgs::DartServoCommand& trigger_command;

    // limit servo
    rmcs_msgs::DartServoCommand& limiting_command;

    // carriage
    rmcs_msgs::DartMechanismCommand& carriage_command;
    double& carriage_target_velocity;

    // yaw pitch force
    int32_t& force_error;
    double& force_max_velocity_override;
    double& force_max_torque_override;
    Eigen::Vector2d& angle_error_vector;
};

struct ManagerSettings {
    // belt
    double belt_down_setting_velocity;
    double belt_down_travel_angle;
    double belt_up_setting_velocity;
    double belt_up_travel_angle;
    double belt_init_setting_velocity;
    double belt_stall_velocity_threshold;
    double belt_stall_torque_threshold;
    uint64_t belt_stall_confirm_ticks;
    double belt_init_stall_velocity_threshold;
    double belt_init_stall_torque_threshold;
    uint64_t belt_init_stall_confirm_ticks;
    double belt_init_max_torque;
    double belt_manual_setting_velocity;

    // lift
    double lift_target_velocity;
    double lift_stall_velocity_threshold;
    double lift_stall_torque_threshold;
    uint64_t lift_stall_confirm_ticks;

    // carriage
    double carriage_down_setting_velocity;
    double carriage_travel_distance;
    double carriage_up_setting_velocity;
    double carriage_adjust_down_distance;
    double carriage_adjust_up_distance;
    double carriage_stall_velocity_threshold;
    double carriage_stall_torque_threshold;
    uint64_t carriage_stall_confirm_ticks;
    double carriage_calibration_setting_velocity;
    double carriage_calibration_stall_velocity_threshold;
    double carriage_calibration_stall_torque_threshold;
    uint64_t carriage_calibration_stall_confirm_ticks;
    double carriage_calibration_max_torque;

    // trigger

    // limit servo
    uint64_t limiting_fill_ticks;

    // yaw pitch force
    int32_t force_setpoint;
    int32_t force_allowable_error;
    double manual_angle_max_error;
    int32_t manual_force_max_error;
};

struct ManagerRuntimeState {
    uint32_t fire_count{0};
    ManagerLifecycleState lifecycle_state{ManagerLifecycleState::IDLE};
    std::optional<double> carriage_power_cycle_origin_angle;
};

struct ManagerQueuedTaskInfo {
    std::string task_name;
    std::string display_name;

    friend bool operator==(const ManagerQueuedTaskInfo&, const ManagerQueuedTaskInfo&) = default;
};

struct ManagerLastErrorInfo {
    std::string task_name;
    std::string action_name;
    ActionFailureReason reason{ActionFailureReason::NONE};
    int64_t timestamp_ms{0};

    friend bool operator==(const ManagerLastErrorInfo&, const ManagerLastErrorInfo&) = default;
};

} // namespace rmcs_dart_guidance::manager
