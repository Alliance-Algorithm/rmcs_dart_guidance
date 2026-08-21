#pragma once

#include "manager/runtime/task.hpp"
#include <rmcs_dart_guidance/action/chassis_command_action.hpp>
#include <rmcs_dart_guidance/resource/mechanism_resources.hpp>

#include <memory>

#include <rmcs_dart_guidance/msg/chassis_command.hpp>

namespace rmcs_dart_guidance::manager {

class DartChassisLevelTask : public Task {
public:
    explicit DartChassisLevelTask(MechanismResources& resources)
        : Task("dart-chassis-level", "4z chassis level") {
        using rmcs_dart_guidance::msg::ChassisCommand;

        then(
            std::make_shared<ChassisCommandAction>(
                "chassis_zero_calibrate", resources.chassis, ChassisCommand::ZERO_CALIBRATE,
                kMechanismTimeoutTicks));

        then(
            std::make_shared<ChassisCommandAction>(
                "chassis_level", resources.chassis, ChassisCommand::LEVEL, kMechanismTimeoutTicks));
    }

private:
    static constexpr uint64_t kMechanismTimeoutTicks = 60000;
};

} // namespace rmcs_dart_guidance::manager
