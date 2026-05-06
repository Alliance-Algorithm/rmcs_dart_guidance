#pragma once

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/vision_aim_profile_provider.hpp"

#include <memory>
#include <string>

namespace rmcs_dart_guidance::manager {

std::shared_ptr<Task> make_belt_init_task(
    const ManagerInputContext& input, ManagerOutputContext& output,
    const ManagerSettings& settings);

std::shared_ptr<Task> make_task(
    const std::string& cmd, const ManagerInputContext& input, ManagerOutputContext& output,
    const ManagerSettings& settings, const VisionAimProfileProvider& profile_provider,
    ManagerRuntimeState& runtime_state);

} // namespace rmcs_dart_guidance::manager
