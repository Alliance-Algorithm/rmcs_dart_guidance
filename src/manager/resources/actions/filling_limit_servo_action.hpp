#pragma once

#include "manager/core/runtime/action.hpp"

#include <cstdint>
#include <string>

#include "rmcs_msgs/dart_servo_command.hpp"

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// FillingLimitServoAction
//   填装限位舵机定时动作：进入时先下发触发位命令，在 fill_ticks 到期后自动切回
//   锁定位并返回 SUCCESS。即使动作被中断，on_exit 也会再次写入锁定位，避免舵机
//   残留在临时状态。
// ─────────────────────────────────────────────────────────────────────────────
class FillingLimitServoAction : public IAction {
public:
    FillingLimitServoAction(
        std::string name,                              //
        rmcs_msgs::DartServoCommand& limiting_command, //
        rmcs_msgs::DartServoCommand trigger_command,   //
        rmcs_msgs::DartServoCommand lock_command,      //
        uint64_t fill_ticks                            //
        )
        : IAction(std::move(name))
        , limiting_command_(limiting_command)
        , trigger_command_(trigger_command)
        , lock_command_(lock_command)
        , fill_ticks_(fill_ticks) {}

    void on_enter() override { limiting_command_ = trigger_command_; }

    ActionStatus update() override {
        if (elapsed_ticks() >= fill_ticks_) {
            limiting_command_ = lock_command_;
            return ActionStatus::SUCCESS;
        }
        return ActionStatus::RUNNING;
    }

    void on_exit() override { limiting_command_ = lock_command_; }

private:
    rmcs_msgs::DartServoCommand& limiting_command_;
    rmcs_msgs::DartServoCommand trigger_command_;
    rmcs_msgs::DartServoCommand lock_command_;
    uint64_t fill_ticks_;
};

} // namespace rmcs_dart_guidance::manager
