#pragma once

#include "manager/core/runtime/action.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <string>

#include <utility>

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// ForceControlAction
//   力度闭环上层动作：根据两路力传感器反馈均值实时计算目标力度误差，并持续发布给
//   下层控制器。误差进入允许范围后返回 SUCCESS；若超过超时窗口仍未收敛则返回
//   TIMEOUT。退出时会将误差清零。
// ─────────────────────────────────────────────────────────────────────────────
class ForceControlAction : public IAction {
public:
    ForceControlAction(
        std::string name,                           //
        int32_t& force_error_interface,             //
        const int32_t& force_feedback_ch1_interface, //
        const int32_t& force_feedback_ch2_interface, //
        int32_t force_setting,                      //
        int32_t allowable_error,                    //
        uint64_t timeout_ticks_setting              //
        )
        : IAction(std::move(name))
        , force_error_output_interface_(force_error_interface)
        , force_feedback_ch1_input_interface_(force_feedback_ch1_interface)
        , force_feedback_ch2_input_interface_(force_feedback_ch2_interface)
        , target_force_(force_setting)
        , allowable_error_(allowable_error)
        , timeout_ticks_(timeout_ticks_setting) {}

    void on_enter() override { force_error_output_interface_ = compute_force_error(); }

    ActionStatus update() override {
        if (elapsed_ticks() >= timeout_ticks_) {
            return fail(ActionFailureReason::TIMEOUT);
        }

        force_error_output_interface_ = compute_force_error();
        if (std::llabs(static_cast<int64_t>(force_error_output_interface_))
            < static_cast<int64_t>(allowable_error_)) {
            return ActionStatus::SUCCESS;
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override { force_error_output_interface_ = 0; }

private:
    int32_t compute_force_error() const {
        const int64_t avg_force_feedback =
            (static_cast<int64_t>(force_feedback_ch1_input_interface_)
             + static_cast<int64_t>(force_feedback_ch2_input_interface_))
            / 2;
        const int64_t force_error = static_cast<int64_t>(target_force_) - avg_force_feedback;

        return static_cast<int32_t>(std::clamp(
            force_error,
            static_cast<int64_t>(std::numeric_limits<int32_t>::min()),
            static_cast<int64_t>(std::numeric_limits<int32_t>::max())));
    }

    int32_t& force_error_output_interface_;

    const int32_t& force_feedback_ch1_input_interface_;
    const int32_t& force_feedback_ch2_input_interface_;

    int32_t target_force_;
    int32_t allowable_error_;
    uint64_t timeout_ticks_;
};

} // namespace rmcs_dart_guidance::manager
