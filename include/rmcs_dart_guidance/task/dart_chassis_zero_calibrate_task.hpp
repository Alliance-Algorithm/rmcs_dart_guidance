#pragma once

#include "manager/runtime/task.hpp"
#include <rmcs_dart_guidance/action/chassis_command_action.hpp>
#include <rmcs_dart_guidance/resource/mechanism_resources.hpp>

#include <memory>

#include <rmcs_dart_guidance/msg/chassis_command.hpp>

namespace rmcs_dart_guidance::manager {

class DartChassisZeroCalibrateTask : public Task {
public:
    explicit DartChassisZeroCalibrateTask(MechanismResources& resources)
        : Task("dart-chassis-zero-calibrate", "4z chassis zero calibrate") {
        using rmcs_dart_guidance::msg::ChassisCommand;

        then(
            std::make_shared<ChassisCommandAction>(
                "chassis_zero_calibrate", resources.chassis, ChassisCommand::ZERO_CALIBRATE,
                kMechanismTimeoutTicks));
    }

private:
    static constexpr uint64_t kMechanismTimeoutTicks = 20000;
};

} // namespace rmcs_dart_guidance::manager
