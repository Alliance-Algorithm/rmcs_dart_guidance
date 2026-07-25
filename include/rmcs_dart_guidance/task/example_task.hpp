#pragma once

#include <rmcs_dart_guidance/action/delay_action.hpp>
#include <rmcs_dart_guidance/action/example_command_action.hpp>
#include "manager/runtime/task.hpp"
#include <rmcs_dart_guidance/resource/example_resource.hpp>

#include <memory>

#include <rmcs_dart_guidance/msg/example_command.hpp>

namespace rmcs_dart_guidance::manager {

class ExampleTask : public Task {
public:
    explicit ExampleTask(ExampleResource& resource)
        : Task("example", "示例任务") {
        using rmcs_dart_guidance::msg::ExampleCommand;

        then(std::make_shared<ExampleCommandAction>(
            "example_1", resource, ExampleCommand::RUNNING, 5000));
        then(std::make_shared<DelayAction>("example_delay", 1000));
        then(std::make_shared<ExampleCommandAction>(
            "example_2", resource, ExampleCommand::RUNNING, 5000));
    }
};

} // namespace rmcs_dart_guidance::manager
