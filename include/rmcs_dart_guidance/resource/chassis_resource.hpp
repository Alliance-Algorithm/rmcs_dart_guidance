#pragma once

#include <string>

#include <rclcpp/logging.hpp>
#include <rmcs_dart_guidance/msg/chassis_command.hpp>
#include <rmcs_dart_guidance/msg/mechanism_status.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_dart_guidance::manager {

class ChassisResource {
public:
    ChassisResource(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix = "/dart/chassis") {
        command_component.register_output(
            name_prefix + "/command", command_, rmcs_dart_guidance::msg::ChassisCommand::IDLE);
        status_component.register_input(name_prefix + "/status", status_, false);
        status_name_ = name_prefix + "/status";
    }

    ChassisResource(const ChassisResource&) = delete;
    ChassisResource& operator=(const ChassisResource&) = delete;
    ChassisResource(ChassisResource&&) = delete;
    ChassisResource& operator=(ChassisResource&&) = delete;

    void bind_optional(const rclcpp::Logger& logger) {
        if (!status_.ready()) {
            status_.make_and_bind_directly(rmcs_dart_guidance::msg::MechanismStatus::IDLE);
            RCLCPP_WARN(logger, "Failed to fetch \"%s\". Set to IDLE.", status_name_.c_str());
        }
    }

    void request(rmcs_dart_guidance::msg::ChassisCommand c) { *command_ = c; }
    void idle() { *command_ = rmcs_dart_guidance::msg::ChassisCommand::IDLE; }
    void abort() { *command_ = rmcs_dart_guidance::msg::ChassisCommand::ABORT; }

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
    rmcs_executor::Component::OutputInterface<rmcs_dart_guidance::msg::ChassisCommand> command_;
    rmcs_executor::Component::InputInterface<rmcs_dart_guidance::msg::MechanismStatus> status_;
    std::string status_name_;
};

} // namespace rmcs_dart_guidance::manager
