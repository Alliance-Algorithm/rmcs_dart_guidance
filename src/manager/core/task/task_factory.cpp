#include "manager/core/task/task_factory.hpp"

#include "manager/core/task/cancel_launch_task.hpp"
#include "manager/core/task/fire_and_preload_task.hpp"
#include "manager/core/task/launch_preparation_task.hpp"

namespace rmcs_dart_guidance::manager {

std::shared_ptr<Task> make_task(
    const std::string& cmd, MechanismResources& resources, ManagerRuntimeState& runtime_state) {
    if (cmd == "launch_prepare" || cmd == "launch-prepare") {
        return std::make_shared<LaunchPreparationTask>(resources, runtime_state);
    }
    if (cmd == "fire_preload" || cmd == "fire") {
        return std::make_shared<FireAndPreloadTask>(resources, runtime_state);
    }
    if (cmd == "launch_cancel" || cmd == "cancel_launch" || cmd == "unload") {
        return std::make_shared<CancelLaunchTask>(resources);
    }
    return nullptr;
}

} // namespace rmcs_dart_guidance::manager
