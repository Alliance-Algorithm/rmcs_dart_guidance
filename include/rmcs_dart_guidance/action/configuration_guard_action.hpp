#pragma once

#include "manager/runtime/action.hpp"

#include <string>
#include <utility>

namespace rmcs_dart_guidance::manager {

class ConfigurationGuardAction : public IAction {
public:
    ConfigurationGuardAction(std::string name, bool condition)
        : IAction(std::move(name))
        , condition_(condition) {}

    ActionStatus update() override {
        if (!condition_) {
            return fail(ActionFailureReason::CONFIGURATION_ERROR);
        }
        return ActionStatus::SUCCESS;
    }

private:
    bool condition_;
};

} // namespace rmcs_dart_guidance::manager
