#pragma once

#include "manager/runtime/task.hpp"
#include <rmcs_dart_guidance/action/trigger_command_action.hpp>
#include <rmcs_dart_guidance/resource/mechanism_resources.hpp>

#include <memory>

#include <rmcs_dart_guidance/msg/trigger_command.hpp>

namespace rmcs_dart_guidance::manager {

class DartCarriageCalibrateTask : public Task {
public:
    explicit DartCarriageCalibrateTask(MechanismResources& resources)
        : Task("dart-carriage-calibrate", "滑台标定") {
        using rmcs_dart_guidance::msg::TriggerCommand;

        then(
            std::make_shared<TriggerCommandAction>(
                "carriage_calibrate", resources.trigger, TriggerCommand::CARRIAGE_CALIBRATE,
                kMechanismTimeoutTicks));
    }

private:
    static constexpr uint64_t kMechanismTimeoutTicks = 5000;
};

} // namespace rmcs_dart_guidance::manager
