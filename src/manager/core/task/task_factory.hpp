#pragma once

#include "manager/core/runtime/task.hpp"
#include "manager/resources/example_resource.hpp"

#include <memory>
#include <string>

namespace rmcs_dart_guidance::manager {

std::shared_ptr<Task> make_task(const std::string& cmd, ExampleResource& example_resource);

} // namespace rmcs_dart_guidance::manager
