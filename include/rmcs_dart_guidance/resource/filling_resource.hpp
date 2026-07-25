#pragma once

#include <string>

#include <rclcpp/logging.hpp>
#include <rmcs_dart_guidance/msg/filling_command.hpp>
#include <rmcs_dart_guidance/msg/mechanism_status.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_dart_guidance::manager {

class FillingResource {
public:
    FillingResource(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix = "/dart/filling") {
        command_component.register_output(
            name_prefix + "/command", command_, rmcs_dart_guidance::msg::FillingCommand::IDLE);
        status_component.register_input(name_prefix + "/status", status_, false);
        status_name_ = name_prefix + "/status";
    }

    FillingResource(const FillingResource&) = delete;
    FillingResource& operator=(const FillingResource&) = delete;
    FillingResource(FillingResource&&) = delete;
    FillingResource& operator=(FillingResource&&) = delete;

    void bind_optional(const rclcpp::Logger& logger) {
        if (!status_.ready()) {
            status_.make_and_bind_directly(rmcs_dart_guidance::msg::MechanismStatus::IDLE);
            RCLCPP_WARN(logger, "Failed to fetch \"%s\". Set to IDLE.", status_name_.c_str());
        }
    }

    void request(rmcs_dart_guidance::msg::FillingCommand c) { *command_ = c; }
    void idle() { *command_ = rmcs_dart_guidance::msg::FillingCommand::IDLE; }
    void abort() { *command_ = rmcs_dart_guidance::msg::FillingCommand::ABORT; }

    bool succeeded() const {
        return status_.ready() && *status_ == rmcs_dart_guidance::msg::MechanismStatus::SUCCEEDED;
    }
    bool failed() const {
        return status_.ready()
            && (*status_ == rmcs_dart_guidance::msg::MechanismStatus::FAILED
                || *status_ == rmcs_dart_guidance::msg::MechanismStatus::ABORTED);
    }
    bool busy() const {
        return status_.ready() && *status_ == rmcs_dart_guidance::msg::MechanismStatus::BUSY;
    }

private:
    rmcs_executor::Component::OutputInterface<rmcs_dart_guidance::msg::FillingCommand> command_;
    rmcs_executor::Component::InputInterface<rmcs_dart_guidance::msg::MechanismStatus> status_;
    std::string status_name_;
};

} // namespace rmcs_dart_guidance::manager
