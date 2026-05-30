#pragma once

#include "manager/core/runtime/action.hpp"

#include <cstdint>
#include <string>

namespace rmcs_dart_guidance::manager {

class FireCountValidationAction : public IAction {
public:
    FireCountValidationAction(
        std::string name, const uint32_t& fire_count, uint32_t expected_fire_count)
        : IAction(std::move(name))
        , fire_count_(fire_count)
        , expected_fire_count_(expected_fire_count) {}

    ActionStatus update() override {
        if (fire_count_ == expected_fire_count_) {
            return ActionStatus::SUCCESS;
        }

        if (runtime_context().logger != nullptr) {
            RCLCPP_ERROR(
                *runtime_context().logger,
                "[FireCountValidationAction] expected fire_count=%u but got %u",
                expected_fire_count_, fire_count_);
        }
        return fail(ActionFailureReason::INVALID_INPUT);
    }

private:
    const uint32_t& fire_count_;
    uint32_t expected_fire_count_;
};

} // namespace rmcs_dart_guidance::manager
