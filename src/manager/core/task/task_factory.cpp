#include "manager/core/task/task_factory.hpp"

#include "manager/core/task/example_task.hpp"

namespace rmcs_dart_guidance::manager {

std::shared_ptr<Task> make_task(const std::string& cmd, ExampleResource& example_resource) {
    if (cmd == "example") {
        return std::make_shared<ExampleTask>(example_resource);
    }
    return nullptr;
}

} // namespace rmcs_dart_guidance::manager
