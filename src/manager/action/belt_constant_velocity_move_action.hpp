#pragma once

#include "action.hpp"

#include <cmath>
#include <cstdint>
#include <string>

#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

// BeltConstantVelocityMoveAction - 匀速运动到目标位置
//   不使用PID位置控制，直接使用速度控制模式
//   完成条件：到达目标位置（基于多圈角度反馈）
//   优点：避免PID error过大导致的问题，运动更平滑
//
//   重要：目标位置基于初始化后的实际角度计算，而非概念上的"零点"
//   - 输入参数是距离（m），在函数内换算为角度（rad）
//   - 目标位置 = 初始角度 + (目标距离 / 滑轮半径)
class BeltConstantVelocityMoveAction : public IAction {
public:
    BeltConstantVelocityMoveAction(
        std::string name,
        rmcs_msgs::DartSliderStatus& belt_command,
        double& belt_target_velocity,
        double& belt_torque_limit,
        const double& left_belt_angle,
        const double& right_belt_angle,
        const double& left_belt_velocity,
        const double& right_belt_velocity,
        double target_distance,           // 目标距离（m，正=上行，负=下行）
        double pulley_radius,             // 滑轮半径（m）
        double velocity,                  // 运动速度（rad/s）
        double torque_limit,              // 扭矩限制（N⋅m）
        uint64_t timeout_ticks,
        uint64_t min_running_ticks = 50,
        double position_tolerance = 0.01, // 位置到达容差（rad）
        double stall_velocity_threshold = 0.3,
        uint64_t stall_confirm_ticks = 200,
        double max_down_distance = 0.20)  // 下行最大距离限制（m，正值，防止过度下行）
        : IAction(std::move(name))
        , belt_command_(belt_command)
        , belt_target_velocity_(belt_target_velocity)
        , belt_torque_limit_(belt_torque_limit)
        , left_belt_angle_(left_belt_angle)
        , right_belt_angle_(right_belt_angle)
        , left_belt_velocity_(left_belt_velocity)
        , right_belt_velocity_(right_belt_velocity)
        , target_distance_(target_distance)
        , pulley_radius_(pulley_radius)
        , velocity_(velocity)
        , torque_limit_(torque_limit)
        , timeout_ticks_(timeout_ticks)
        , min_running_ticks_(min_running_ticks)
        , position_tolerance_(position_tolerance)
        , stall_velocity_threshold_(stall_velocity_threshold)
        , stall_confirm_ticks_(stall_confirm_ticks)
        , max_down_distance_(max_down_distance) {}

    void on_enter() override {
        stall_counter_ = 0;

        // 读取初始角度（左传送带，右传送带使用同步控制）
        initial_angle_ = left_belt_angle_;

        // 将目标距离（m）换算为角度偏移（rad）
        // angle_offset = distance / radius
        double target_angle_offset = target_distance_ / pulley_radius_;

        // 计算实际目标位置 = 初始角度 + 角度偏移
        target_position_ = initial_angle_ + target_angle_offset;

        // 下行最大距离限制检查（防止过度下行）
        // 将最大下行距离（m）换算为角度偏移（rad）
        double max_down_angle_offset = -max_down_distance_ / pulley_radius_;  // 负值表示下行
        double max_down_position = initial_angle_ + max_down_angle_offset;

        if (target_position_ < max_down_position) {
            printf("[%s] ERROR: target_position=%.4f exceeds max_down_position=%.4f\n",
                   name().c_str(), target_position_, max_down_position);
            printf("[%s] target_distance=%.4f m, max_down_distance=%.4f m\n",
                   name().c_str(), target_distance_, max_down_distance_);
            printf("[%s] Limiting target to max_down_position=%.4f\n",
                   name().c_str(), max_down_position);
            target_position_ = max_down_position;
        }

        // 根据目标位置自动确定运动方向
        // 实际测量：角度增大=下行，角度减小=上行
        double distance_to_target = target_position_ - initial_angle_;
        if (distance_to_target > 0) {
            // 目标位置 > 初始位置，角度增大方向 = 下行
            belt_command_ = rmcs_msgs::DartSliderStatus::DOWN;
            belt_target_velocity_ = std::abs(velocity_);
        } else {
            // 目标位置 < 初始位置，角度减小方向 = 上行
            belt_command_ = rmcs_msgs::DartSliderStatus::UP;
            belt_target_velocity_ = std::abs(velocity_);
        }

        belt_torque_limit_ = torque_limit_;

        printf("[%s] on_enter: initial_angle=%.4f, target_distance=%.4f m, target_angle_offset=%.4f rad\n",
               name().c_str(), initial_angle_, target_distance_, target_angle_offset);
        printf("[%s] target_position=%.4f, velocity=%.4f, direction=%s\n",
               name().c_str(), target_position_, belt_target_velocity_,
               (distance_to_target > 0) ? "DOWN" : "UP");
    }

    ActionStatus update() override {
        if (elapsed_ticks() >= timeout_ticks_) {
            printf("[%s] TIMEOUT after %lu ticks\n", name().c_str(), elapsed_ticks());
            return ActionStatus::FAILURE;
        }

        double avg_angle = (left_belt_angle_ + right_belt_angle_) / 2.0;
        double position_error = target_position_ - avg_angle;  // 保留符号，用于判断方向
        double avg_velocity = (std::abs(left_belt_velocity_) + std::abs(right_belt_velocity_)) / 2.0;

        // 每100帧打印一次调试信息
        if (elapsed_ticks() % 100 == 0) {
            printf("[%s] tick=%lu, current=%.4f, target=%.4f, error=%.4f, vel=%.4f\n",
                   name().c_str(), elapsed_ticks(), avg_angle, target_position_,
                   position_error, avg_velocity);
            printf("[%s]   left_angle=%.4f, right_angle=%.4f, initial=%.4f\n",
                   name().c_str(), left_belt_angle_, right_belt_angle_, initial_angle_);
        }

        // 位置到达判断（优先级最高）
        // 判断条件：
        // 1. 在容差范围内：|error| < tolerance
        // 2. 或者已经越过目标（考虑运动方向）：
        //    - 下行（target > initial）：error < 0 表示已经越过（current > target）
        //    - 上行（target < initial）：error > 0 表示已经越过（current < target）
        if (elapsed_ticks() > min_running_ticks_) {
            bool within_tolerance = std::abs(position_error) < position_tolerance_;
            bool overshot = false;

            if (target_distance_ > 0) {
                // 下行：目标位置 > 初始位置，期望 current 增大
                // 如果 error < 0，说明 current > target，已经越过
                overshot = (position_error < 0);
            } else {
                // 上行：目标位置 < 初始位置，期望 current 减小
                // 如果 error > 0，说明 current < target，已经越过
                overshot = (position_error > 0);
            }

            if (within_tolerance || overshot) {
                printf("[%s] SUCCESS: position reached (error=%.4f, tolerance=%.4f, overshot=%d)\n",
                       name().c_str(), position_error, position_tolerance_, overshot);
                return ActionStatus::SUCCESS;
            }
        }

        // 堵转检测
        if (elapsed_ticks() > min_running_ticks_) {
            if (avg_velocity < stall_velocity_threshold_) {
                ++stall_counter_;
                if (stall_counter_ >= stall_confirm_ticks_) {
                    printf("[%s] FAILURE: stalled (vel=%.4f < threshold=%.4f for %lu ticks)\n",
                           name().c_str(), avg_velocity, stall_velocity_threshold_, stall_counter_);
                    return ActionStatus::FAILURE;
                }
            } else {
                stall_counter_ = 0;
            }
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        // 不在这里停止运动，让下一个 action 接管控制
        // 这样可以避免速度突变
    }

private:
    rmcs_msgs::DartSliderStatus& belt_command_;
    double& belt_target_velocity_;
    double& belt_torque_limit_;
    const double& left_belt_angle_;
    const double& right_belt_angle_;
    const double& left_belt_velocity_;
    const double& right_belt_velocity_;

    double target_distance_;        // 目标距离（m）
    double pulley_radius_;          // 滑轮半径（m）
    double velocity_;               // 运动速度（rad/s）
    double torque_limit_;           // 扭矩限制（N⋅m）
    uint64_t timeout_ticks_;
    uint64_t min_running_ticks_;
    double position_tolerance_;     // 位置到达容差（rad）
    double stall_velocity_threshold_;
    uint64_t stall_confirm_ticks_;
    double max_down_distance_;      // 下行最大距离限制（m，正值）

    double initial_angle_{0.0};     // 初始角度（rad，在on_enter中读取）
    double target_position_{0.0};   // 实际目标位置（rad，在on_enter中计算）
    uint64_t stall_counter_{0};
};

} // namespace rmcs_dart_guidance::manager
