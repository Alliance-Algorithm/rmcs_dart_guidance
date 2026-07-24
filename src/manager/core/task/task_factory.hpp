#pragma once

#include "manager/core/runtime/manager_types.hpp"
#include "manager/core/runtime/task.hpp"
#include "manager/resources/mechanism_resources.hpp"

#include <memory>
#include <string>

namespace rmcs_dart_guidance::manager {

std::shared_ptr<Task> make_task(
    const std::string& cmd, MechanismResources& resources, ManagerRuntimeState& runtime_state);

} // namespace rmcs_dart_guidance::manager
