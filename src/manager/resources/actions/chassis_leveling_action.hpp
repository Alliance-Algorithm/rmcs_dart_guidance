#pragma once

#include "manager/core/runtime/action.hpp"

#include <cmath>
#include <cstdint>
#include <string>
#include <utility>

namespace rmcs_dart_guidance::manager {

class RollLevelingAction : public IAction {
public:
    RollLevelingAction(
        std::string name,                        //
        bool& roll_leveling_flag_interface,      // 发布给 controller 的调平周期 flag
        const double& imu_roll_angle_interface,  // 当前轴 IMU 角度反馈
        const double& front_left_velocity,       // fl电机速度反馈
        const double& rear_left_velocity,        // rl电机速度反馈
        const double& front_left_torque,         // fl电机力矩反馈
        const double& rear_left_torque           // rl电机力矩反馈
        )
        : IAction(std::move(name))
        , roll_leveling_flag_output_interface_(roll_leveling_flag_interface)
        , imu_roll_angle_input_interface_(imu_roll_angle_interface)
        , front_left_velocity_input_interface_(front_left_velocity)
        , rear_left_velocity_input_interface_(rear_left_velocity)
        , front_left_torque_input_interface_(front_left_torque)
        , rear_left_torque_input_interface_(rear_left_torque) {}

    void on_enter() override {
        roll_leveling_flag_output_interface_ = true;
        stall_counter_ = 0;
        confirm_counter_ = 0;
    }

    ActionStatus update() override {
        if (elapsed_ticks() >= 100) {
            miner_velocity = std::min(
                std::abs(front_left_velocity_input_interface_),
                std::abs(rear_left_velocity_input_interface_));
            bigger_torque = std::max(
                std::abs(front_left_torque_input_interface_),
                std::abs(rear_left_torque_input_interface_));
            if (miner_velocity <= 1.0 && bigger_torque >= 0.5) {
                ++stall_counter_;
                if (stall_counter_ >= 500) {
                    return fail(ActionFailureReason::STALL);
                }
            }

            if (std::abs(imu_roll_angle_input_interface_) <= 0.01) {
                ++confirm_counter_;
                if (confirm_counter_ >= 500) {
                    return ActionStatus::SUCCESS;
                }
            }
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override { roll_leveling_flag_output_interface_ = false; }

private:
    bool& roll_leveling_flag_output_interface_;
    const double& imu_roll_angle_input_interface_;
    const double& front_left_velocity_input_interface_;
    const double& rear_left_velocity_input_interface_;
    const double& front_left_torque_input_interface_;
    const double& rear_left_torque_input_interface_;
    double miner_velocity;
    double bigger_torque;

    uint64_t stall_counter_{0};
    uint64_t confirm_counter_{0};
};

class PitchLevelingAction : public IAction {
public:
    PitchLevelingAction(
        std::string name,                        //
        bool& pitch_leveling_flag_interface,     // 发布给 controller 的调平周期 flag
        const double& imu_pitch_angle_interface, // 当前轴 IMU 角度反馈
        const double& front_left_velocity,       // fl电机速度反馈
        const double& front_left_troque,         // rl电机速度反馈
        const double& front_right_velocity,      // fl电机力矩反馈
        const double& front_right_torque         // rl电机力矩反馈
        )
        : IAction(std::move(name))
        , pitch_leveling_flag_output_interface_(pitch_leveling_flag_interface)
        , imu_pitch_angle_input_interface_(imu_pitch_angle_interface)
        , front_left_velocity_input_interface_(front_left_velocity)
        , front_right_velocity_input_interface_(front_right_velocity)
        , front_left_torque_input_interface_(front_left_troque)
        , front_right_torque_input_interface_(front_right_torque) {}

    void on_enter() override {
        pitch_leveling_flag_output_interface_ = true;
        stall_counter_ = 0;
        confirm_counter_ = 0;
    }

    ActionStatus update() override {
        if (elapsed_ticks() >= 100) {
            miner_velocity = std::min(
                std::abs(front_left_velocity_input_interface_),
                std::abs(front_right_velocity_input_interface_));
            bigger_torque = std::max(
                std::abs(front_left_torque_input_interface_),
                std::abs(front_right_torque_input_interface_));
            if (miner_velocity <= 1.0 && bigger_torque >= 0.5) {
                ++stall_counter_;
                if (stall_counter_ >= 500) {
                    return fail(ActionFailureReason::STALL);
                }
            }

            if (std::abs(imu_pitch_angle_input_interface_) <= 0.01) {
                ++confirm_counter_;
                if (confirm_counter_ >= 500) {
                    return ActionStatus::SUCCESS;
                }
            }
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override { pitch_leveling_flag_output_interface_ = false; }

private:
    bool& pitch_leveling_flag_output_interface_;
    const double& imu_pitch_angle_input_interface_;
    const double& front_left_velocity_input_interface_;
    const double& front_right_velocity_input_interface_;
    const double& front_left_torque_input_interface_;
    const double& front_right_torque_input_interface_;
    double miner_velocity;
    double bigger_torque;

    uint64_t stall_counter_{0};
    uint64_t confirm_counter_{0};
};
} // namespace rmcs_dart_guidance::manager