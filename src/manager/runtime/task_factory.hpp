#pragma once

#include "manager/runtime/manager_types.hpp"
#include "manager/runtime/task.hpp"

#include <rmcs_dart_guidance/resource/mechanism_resources.hpp>
#include <rmcs_dart_guidance/task/cancel_launch_task.hpp>
#include <rmcs_dart_guidance/task/fire_and_preload_task.hpp>
#include <rmcs_dart_guidance/task/launch_preparation_task.hpp>

#include <memory>
#include <string>

namespace rmcs_dart_guidance::manager {

inline std::shared_ptr<Task> make_task(
    const std::string& cmd, MechanismResources& mechanism_resources,
    ManagerRuntimeState& runtime_state) {
    if (cmd == "launch_prepare" || cmd == "launch-prepare") {
        return std::make_shared<LaunchPreparationTask>(mechanism_resources, runtime_state);
    }
    if (cmd == "fire_preload" || cmd == "fire") {
        return std::make_shared<FireAndPreloadTask>(mechanism_resources, runtime_state);
    }
    if (cmd == "launch_cancel" || cmd == "cancel_launch" || cmd == "unload") {
        return std::make_shared<CancelLaunchTask>(mechanism_resources);
    }
    return nullptr;
}

} // namespace rmcs_dart_guidance::manager
