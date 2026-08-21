#pragma once

#include "manager/runtime/manager_types.hpp"
#include "manager/runtime/task.hpp"
#include <rmcs_dart_guidance/action/configuration_guard_action.hpp>
#include <rmcs_dart_guidance/action/yaw_command_action.hpp>
#include <rmcs_dart_guidance/resource/mechanism_resources.hpp>

#include <algorithm>
#include <cstddef>
#include <memory>

#include <rmcs_dart_guidance/msg/yaw_command.hpp>

namespace rmcs_dart_guidance::manager {

class DartVisionAimTask : public Task {
public:
    DartVisionAimTask(
        MechanismResources& resources, const ManagerRuntimeState& runtime_state,
        const ManagerSettings& settings)
        : Task("vision-aim", "视觉瞄准") {
        using rmcs_dart_guidance::msg::YawCommand;

        const std::size_t setpoint_index =
            std::min<std::size_t>(static_cast<std::size_t>(runtime_state.fire_count), 3);

        then(
            std::make_shared<ConfigurationGuardAction>(
                "vision_aim_target_setpoints_configured",
                settings.vision_aim_target_setpoints_configured));
        then(
            std::make_shared<YawCommandAction>(
                "vision-aim", resources.yaw, YawCommand::VISION_AIM, kAimTimeoutTicks,
                settings.vision_aim_target_setpoints[setpoint_index]));
    }

private:
    static constexpr uint64_t kAimTimeoutTicks = 100000;
};

} // namespace rmcs_dart_guidance::manager
