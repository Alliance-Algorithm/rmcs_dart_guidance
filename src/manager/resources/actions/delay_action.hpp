#pragma once

#include "manager/core/runtime/action.hpp"

#include <cstdint>
#include <string>
#include <utility>

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// DelayAction
//   纯时间延迟动作：不产生任何控制输出，仅等待指定 tick 数后返回 SUCCESS。
//   常用于在顺序动作之间插入稳定窗口或给下层执行器预留响应时间。
// ─────────────────────────────────────────────────────────────────────────────
class DelayAction : public IAction {
public:
    DelayAction(std::string name, uint64_t delay_ticks)
        : IAction(std::move(name))
        , delay_ticks_(delay_ticks) {}

    ActionStatus update() override {
        if (elapsed_ticks() >= delay_ticks_) {
            return ActionStatus::SUCCESS;
        }
        return ActionStatus::RUNNING;
    }

private:
    uint64_t delay_ticks_;
};

} // namespace rmcs_dart_guidance::manager
