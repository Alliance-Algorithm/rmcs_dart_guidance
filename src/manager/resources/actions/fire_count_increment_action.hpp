#pragma once

#include "manager/core/runtime/action.hpp"

#include <cstdint>
#include <string>

namespace rmcs_dart_guidance::manager {

class FireCountIncrementAction : public IAction {
public:
    FireCountIncrementAction(std::string name, uint32_t& fire_count)
        : IAction(std::move(name))
        , fire_count_(fire_count) {}

    ActionStatus update() override {
        ++fire_count_;
        if (runtime_context().logger != nullptr) {
            RCLCPP_INFO(
                *runtime_context().logger, "[FireCountIncrementAction] fire_count=%u", fire_count_);
        }
        return ActionStatus::SUCCESS;
    }

private:
    uint32_t& fire_count_;
};

} // namespace rmcs_dart_guidance::manager
