#pragma once

#include "manager/core/runtime/action.hpp"

#include <cstdint>
#include <string>
#include <utility>

namespace rmcs_dart_guidance::manager {

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
