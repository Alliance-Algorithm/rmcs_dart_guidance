#pragma once

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/trigger_control_action.hpp"

#include <memory>

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// TriggerLockTask
//   扳机锁定任务：封装一次 TriggerControlAction，将扳机切换到 LOCK 并等待舵机稳定。
// ─────────────────────────────────────────────────────────────────────────────
class TriggerLockTask : public Task {
public:
    explicit TriggerLockTask(ManagerOutputContext& output)
        : Task("trigger_lock", "扳机锁定") {

        then(
            std::make_shared<TriggerControlAction>(
                "trigger_lock",                    // 动作名称
                output.trigger_command,            // 扳机命令接口
                rmcs_msgs::DartServoCommand::LOCK, // 扳机锁定命令
                1000));                            // 舵机稳定等待 tick
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// TriggerFreeTask
//   扳机释放任务：封装一次 TriggerControlAction，将扳机切换到 FREE 并等待动作完成。
// ─────────────────────────────────────────────────────────────────────────────
class TriggerFreeTask : public Task {
public:
    explicit TriggerFreeTask(ManagerOutputContext& output)
        : Task("trigger_free", "扳机释放") {

        then(
            std::make_shared<TriggerControlAction>(
                "trigger_free",                    // 动作名称
                output.trigger_command,            // 扳机命令接口
                rmcs_msgs::DartServoCommand::FREE, // 扳机释放命令
                100));                             // 舵机稳定等待 tick
    }
};

} // namespace rmcs_dart_guidance::manager
