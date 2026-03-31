#pragma once

#include "action.hpp"

namespace rmcs_dart_guidance::manager {

class ZeroCalibrationAction : public IAction {
public:
    explicit ZeroCalibrationAction(bool& zero_calibration_flag, uint64_t hold_ticks = 10)
        : IAction("zero_calibration")
        , zero_calibration_flag_(zero_calibration_flag)
        , hold_ticks_(hold_ticks) {}

    void on_enter() override {
        zero_calibration_flag_ = true;
        tick_count_ = 0;
    }

    ActionStatus update() override {
        ++tick_count_;
        return (tick_count_ >= hold_ticks_) ? ActionStatus::SUCCESS : ActionStatus::RUNNING;
    }

    void on_exit() override { zero_calibration_flag_ = false; }

private:
    bool& zero_calibration_flag_;
    uint64_t hold_ticks_;
    uint64_t tick_count_{0};
};
} // namespace rmcs_dart_guidance::manager