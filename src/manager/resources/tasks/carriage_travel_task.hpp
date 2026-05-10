#pragma once

#include <memory>
#include <string>

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/carriage_control_action.hpp"

namespace rmcs_dart_guidance::manager {

class CarriageTravelTask : public Task {
public:
    CarriageTravelTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings)
        : CarriageTravelTask(
              "carriage_travel", "发射滑台运动", input, output,
              rmcs_msgs::DartMechanismCommand::DOWN, settings.carriage_down_setting_velocity,
              settings.carriage_down_travel_angle, settings) {}

    CarriageTravelTask(
        std::string name, std::string description, const ManagerInputContext& input,
        ManagerOutputContext& output, rmcs_msgs::DartMechanismCommand command_setting,
        double velocity_setting, double travel_angle_setting, const ManagerSettings& settings)
        : Task(std::move(name), std::move(description)) {
        then(
            std::make_shared<CarriageTravelAction>(
                "carriage_travel",                          //
                output.carriage_command,                    //
                output.carriage_target_velocity,            //
                input.carriage_angle,                       //
                input.carriage_origin_angle,                //
                input.carriage_velocity,                    //
                input.carriage_torque,                      //
                command_setting,                            //
                velocity_setting,                           //
                travel_angle_setting,                       //
                settings.carriage_stall_velocity_threshold, //
                settings.carriage_stall_torque_threshold,   //
                settings.carriage_stall_confirm_ticks));    //
        then(
            std::make_shared<CarriageAngleCloseLoopAction>(
                "carriage_angle_close_loop",              //
                output.carriage_command,                  //
                output.carriage_target_velocity,          //
                output.carriage_target_angle,             //
                input.carriage_angle,                     //
                input.carriage_origin_angle,              //
                command_setting,                          //
                velocity_setting,                         //
                travel_angle_setting,                     //
                settings.carriage_angle_allowable_error,  //
                settings.carriage_min_run_ticks,          //
                settings.carriage_timeout_ticks));        //
    }
};

} // namespace rmcs_dart_guidance::manager
