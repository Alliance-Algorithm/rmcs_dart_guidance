#pragma once

#include "manager/runtime/task.hpp"

#include <rmcs_dart_guidance/resource/example_resource.hpp>
#include <rmcs_dart_guidance/task/example_task.hpp>

#include <memory>
#include <string>

namespace rmcs_dart_guidance::manager {

inline std::shared_ptr<Task> make_task(const std::string& cmd, ExampleResource& example_resource) {
    if (cmd == "example") {
        return std::make_shared<ExampleTask>(example_resource);
    }
    return nullptr;
}

} // namespace rmcs_dart_guidance::manager
