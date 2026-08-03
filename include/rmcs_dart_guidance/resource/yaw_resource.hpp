#pragma once

#include <limits>
#include <string>

#include <rclcpp/logging.hpp>
#include <rmcs_dart_guidance/msg/mechanism_status.hpp>
#include <rmcs_dart_guidance/msg/yaw_command.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_dart_guidance::manager {

class YawResource {
public:
    YawResource(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix = "/dart/yaw") {
        command_component.register_output(
            name_prefix + "/command", command_, rmcs_dart_guidance::msg::YawCommand::IDLE);
        command_component.register_output(
            name_prefix + "/target-setpoint", target_setpoint_,
            std::numeric_limits<double>::quiet_NaN());
        status_component.register_input(name_prefix + "/status", status_, false);
        status_name_ = name_prefix + "/status";
    }

    YawResource(const YawResource&) = delete;
    YawResource& operator=(const YawResource&) = delete;
    YawResource(YawResource&&) = delete;
    YawResource& operator=(YawResource&&) = delete;

    void bind_optional(const rclcpp::Logger& logger) {
        if (!status_.ready()) {
            status_.make_and_bind_directly(rmcs_dart_guidance::msg::MechanismStatus::IDLE);
            RCLCPP_WARN(logger, "Failed to fetch \"%s\". Set to IDLE.", status_name_.c_str());
        }
    }

    void request(
        rmcs_dart_guidance::msg::YawCommand c,
        double target_setpoint = std::numeric_limits<double>::quiet_NaN()) {
        *command_ = c;
        *target_setpoint_ = target_setpoint;
    }
    void idle() {
        *command_ = rmcs_dart_guidance::msg::YawCommand::IDLE;
        *target_setpoint_ = std::numeric_limits<double>::quiet_NaN();
    }
    void abort() {
        *command_ = rmcs_dart_guidance::msg::YawCommand::ABORT;
        *target_setpoint_ = std::numeric_limits<double>::quiet_NaN();
    }

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
    rmcs_executor::Component::OutputInterface<rmcs_dart_guidance::msg::YawCommand> command_;
    rmcs_executor::Component::OutputInterface<double> target_setpoint_;
    rmcs_executor::Component::InputInterface<rmcs_dart_guidance::msg::MechanismStatus> status_;
    std::string status_name_;
};

} // namespace rmcs_dart_guidance::manager
