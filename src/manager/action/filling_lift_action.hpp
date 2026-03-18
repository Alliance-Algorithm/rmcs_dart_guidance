#pragma once

#include "action.hpp"

#include <cmath>
#include <cstdint>
#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

// 升降 LK 电机动作（仿 BeltMoveAction 模式）：
//   on_enter: 写入 lifting_command（UP/DOWN），触发 DartLaunchSettingV2 开始速度控制
//   update:   直接读取升降电机速度反馈，检测堵转
//             - 超过 stall_min_run_ticks 后开始检测
//             - 左右任一速度低于 stall_threshold 持续 stall_confirm_ticks →
//             SUCCESS（任一堵转即全部堵转）
//             - 超过 timeout_ticks → FAILURE
class FillingLiftAction : public IAction {
public:
    /// @param name                 动作名称
    /// @param lifting_command      升降指令引用（DartManagerV2 的 lifting_command_ 解引用）
    /// @param command              目标状态（UP / DOWN）
    /// @param left_vel_fb          左升降电机速度反馈（rad/s）
    /// @param right_vel_fb         右升降电机速度反馈（rad/s）
    /// @param stall_threshold      堵转速度阈值（rad/s）
    /// @param stall_confirm_ticks  连续低速帧数确认堵转（100 ticks = 0.1s @ 1kHz）
    /// @param stall_min_run_ticks  启动后最少运行帧数，避免启动瞬间误触发
    /// @param timeout_ticks        超时帧数，超时返回 FAILURE
    /// @param require_motion_before_stall
    ///                              是否要求先观测到电机进入运动，再允许低速堵转判定成功。
    ///                              复位动作可关闭该选项，以便已经在端点时直接判定到位。
    FillingLiftAction(
        const char* name, rmcs_msgs::DartSliderStatus& lifting_command,
        rmcs_msgs::DartSliderStatus command, const double& left_vel_fb, const double& right_vel_fb,
        double stall_threshold, uint64_t stall_confirm_ticks, uint64_t stall_min_run_ticks,
        uint64_t timeout_ticks = 5000, bool require_motion_before_stall = true)
        : IAction(name)
        , lifting_command_(lifting_command)
        , command_(command)
        , left_vel_fb_(left_vel_fb)
        , right_vel_fb_(right_vel_fb)
        , stall_threshold_(stall_threshold)
        , stall_confirm_ticks_(stall_confirm_ticks)
        , stall_min_run_ticks_(stall_min_run_ticks)
        , timeout_ticks_(timeout_ticks)
        , require_motion_before_stall_(require_motion_before_stall) {}

    void on_enter() override {
        lifting_command_ = command_;
        stall_count_ = 0;
        has_started_motion_ = false;
    }

    ActionStatus update() override {
        if (elapsed_ticks() >= timeout_ticks_)
            return ActionStatus::FAILURE;

        const double left_abs_vel = std::abs(left_vel_fb_);
        const double right_abs_vel = std::abs(right_vel_fb_);
        if (left_abs_vel >= stall_threshold_ || right_abs_vel >= stall_threshold_) {
            has_started_motion_ = true;
        }

        if (elapsed_ticks() < stall_min_run_ticks_
            || (require_motion_before_stall_ && !has_started_motion_))
            return ActionStatus::RUNNING;

        // 任一电机堵转即视为全部堵转（避免单侧堵转导致机构歪斜）
        if (left_abs_vel < stall_threshold_ || right_abs_vel < stall_threshold_) {
            ++stall_count_;
        } else {
            stall_count_ = 0;
        }

        return (stall_count_ >= stall_confirm_ticks_) ? ActionStatus::SUCCESS
                                                      : ActionStatus::RUNNING;
    }

    void on_exit() override { lifting_command_ = rmcs_msgs::DartSliderStatus::WAIT; }

private:
    rmcs_msgs::DartSliderStatus& lifting_command_;
    rmcs_msgs::DartSliderStatus command_;
    const double& left_vel_fb_;
    const double& right_vel_fb_;
    double stall_threshold_;
    uint64_t stall_confirm_ticks_;
    uint64_t stall_min_run_ticks_;
    uint64_t timeout_ticks_;
    bool require_motion_before_stall_{true};
    uint64_t stall_count_{0};
    bool has_started_motion_{false};
};

} // namespace rmcs_dart_guidance::manager
