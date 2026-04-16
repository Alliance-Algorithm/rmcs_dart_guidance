#pragma once

#include "action.hpp"

#include <rmcs_msgs/dart_limiting_servo_status.hpp>

namespace rmcs_dart_guidance::manager {

// 填装限位舵机动作：先下发 FREE 语义命令，等待 fill_ticks，再下发 LOCK。
class FillingLimitServoAction : public IAction {
public:
    FillingLimitServoAction(
        rmcs_msgs::DartLimitingServoStatus& limiting_command,
        rmcs_msgs::DartLimitingServoStatus trigger_command,
        rmcs_msgs::DartLimitingServoStatus lock_command)
        : IAction("limiting_fill")
        , limiting_command_(limiting_command)
        , trigger_command_(trigger_command)
        , lock_command_(lock_command) {}

    void on_enter() override { limiting_command_ = trigger_command_; }

    ActionStatus update() override {
        if (elapsed_ticks() >= 200) {
            limiting_command_ = lock_command_;
            return ActionStatus::SUCCESS;
        }
        return ActionStatus::RUNNING;
    }

    void on_exit() override { limiting_command_ = lock_command_; }

private:
    rmcs_msgs::DartLimitingServoStatus& limiting_command_;
    rmcs_msgs::DartLimitingServoStatus trigger_command_;
    rmcs_msgs::DartLimitingServoStatus lock_command_;
};

} // namespace rmcs_dart_guidance::manager
