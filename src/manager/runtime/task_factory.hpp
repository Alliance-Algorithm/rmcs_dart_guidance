#pragma once

#include "manager/runtime/manager_types.hpp"
#include "manager/runtime/task.hpp"

#include <rmcs_dart_guidance/resource/mechanism_resources.hpp>
#include <rmcs_dart_guidance/task/cancel_launch_task.hpp>
#include <rmcs_dart_guidance/task/dart_carriage_calibrate_task.hpp>
#include <rmcs_dart_guidance/task/dart_chassis_level_task.hpp>
#include <rmcs_dart_guidance/task/dart_chassis_zero_calibrate_task.hpp>
#include <rmcs_dart_guidance/task/dart_fire_task.hpp>
#include <rmcs_dart_guidance/task/dart_vision_aim_task.hpp>
#include <rmcs_dart_guidance/task/dart_game_control_task.hpp>
#include <rmcs_dart_guidance/task/dart_init_task.hpp>
#include <rmcs_dart_guidance/task/launch_preparation_task.hpp>

#include <memory>
#include <string>

namespace rmcs_dart_guidance::manager {

inline std::shared_ptr<Task> make_task(
    const std::string& cmd, MechanismResources& mechanism_resources,
    ManagerRuntimeState& runtime_state, const ManagerSettings& settings) {
    if (cmd == "dart-init" || cmd == "dart_init") {
        return std::make_shared<DartInitTask>(mechanism_resources);
    }
    if (cmd == "dart-launch-prepare" || cmd == "dart_launch_prepare" || cmd == "launch_prepare"
        || cmd == "launch-prepare") {
        return std::make_shared<DartLaunchPrepareTask>(mechanism_resources, runtime_state, settings);
    }
    if (cmd == "dart-launch-cancel" || cmd == "dart_launch_cancel" || cmd == "launch_cancel"
        || cmd == "cancel_launch" || cmd == "unload") {
        return std::make_shared<DartLaunchCancelTask>(mechanism_resources);
    }
    if (cmd == "dart-fire" || cmd == "dart_fire" || cmd == "fire_preload" || cmd == "fire") {
        return std::make_shared<DartFireTask>(mechanism_resources, runtime_state, settings);
    }
    if (cmd == "dart-game-control" || cmd == "dart_game_control" || cmd == "game-control"
        || cmd == "game_control") {
        return std::make_shared<DartGameControlTask>(mechanism_resources, runtime_state, settings);
    }
    if (cmd == "yaw-command:vision-aim" || cmd == "vision-aim" || cmd == "vision_aim"
        || cmd == "dart-vision-aim" || cmd == "dart_vision_aim") {
        return std::make_shared<DartVisionAimTask>(mechanism_resources, runtime_state, settings);
    }
    if (cmd == "dart-carriage-calibrate" || cmd == "dart_carriage_calibrate"
        || cmd == "carriage_init" || cmd == "carriage-init") {
        return std::make_shared<DartCarriageCalibrateTask>(mechanism_resources, settings);
    }
    if (cmd == "dart-chassis-zero-calibrate" || cmd == "dart_chassis_zero_calibrate"
        || cmd == "chassis-zero-calibrate" || cmd == "chassis_zero_calibrate"
        || cmd == "zero_calibrate") {
        return std::make_shared<DartChassisZeroCalibrateTask>(mechanism_resources);
    }
    if (cmd == "dart-chassis-level" || cmd == "dart_chassis_level" || cmd == "chassis-level"
        || cmd == "chassis_level" || cmd == "level") {
        return std::make_shared<DartChassisLevelTask>(mechanism_resources);
    }
    return nullptr;
}

} // namespace rmcs_dart_guidance::manager
