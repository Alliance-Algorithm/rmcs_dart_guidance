#include "manager/resources/task_factory.hpp"

#include "manager/resources/tasks/belt_init_task.hpp"
#include "manager/resources/tasks/cancel_launch_task.hpp"
#include "manager/resources/tasks/carriage_init_task.hpp"
#include "manager/resources/tasks/carriage_travel_task.hpp"
#include "manager/resources/tasks/chassis_leveling_task.hpp"
#include "manager/resources/tasks/dart_init_task.hpp"
#include "manager/resources/tasks/filling_lift_task.hpp"
#include "manager/resources/tasks/fire_and_preload_task.hpp"
#include "manager/resources/tasks/launch_preparation_task.hpp"
#include "manager/resources/tasks/launch_preparation_with_vision_task.hpp"
#include "manager/resources/tasks/manual_control_task.hpp"
#include "manager/resources/tasks/trigger_control_task.hpp"

namespace rmcs_dart_guidance::manager {
std::shared_ptr<Task> make_dart_init_task(
    const ManagerInputContext& input, ManagerOutputContext& output,
    const ManagerSettings& settings) {
    return std::make_shared<DartInitTask>(input, output, settings);
}
std::shared_ptr<Task> make_belt_init_task(
    const ManagerInputContext& input, ManagerOutputContext& output,
    const ManagerSettings& settings) {
    return std::make_shared<BeltInitTask>(input, output, settings);
}

std::shared_ptr<Task> make_carriage_calibration_task(
    const ManagerInputContext& input, ManagerOutputContext& output, const ManagerSettings& settings,
    ManagerRuntimeState& runtime_state) {
    return std::make_shared<CarriageCalibrationTask>(input, output, settings, runtime_state);
}

std::shared_ptr<Task>
    make_chassis_leveling_task(const ManagerInputContext& input, ManagerOutputContext& output) {
    return std::make_shared<ChassisLevelingTask>(input, output);
}

std::shared_ptr<Task> make_carriage_travel_task(
    const ManagerInputContext& input, ManagerOutputContext& output,
    const ManagerSettings& settings) {
    return std::make_shared<CarriageTravelTask>(input, output, settings);
}

std::shared_ptr<Task> make_task(
    const std::string& cmd, const ManagerInputContext& input, ManagerOutputContext& output,
    const ManagerSettings& settings, const VisionAimProfileProvider& profile_provider,
    ManagerRuntimeState& runtime_state) {
    if (cmd == "belt_init" || cmd == "belt-init") {
        return std::make_shared<BeltInitTask>(input, output, settings);
    }

    if (cmd == "carriage_init" || cmd == "carriage-init") {
        return make_carriage_calibration_task(input, output, settings, runtime_state);
    }

    if (cmd == "launch_prepare" || cmd == "launch-prepare") {
        return std::make_shared<LaunchPreparationTask>(input, output, settings, runtime_state);
    }

    if (cmd == "launch_prepare_with_vision" || cmd == "launch-prepare-with-vision") {
        return std::make_shared<LaunchPreparationWithVisionTask>(
            input, output, settings, profile_provider, runtime_state);
    }

    if (cmd == "launch_cancel" || cmd == "cancel_launch" || cmd == "unload") {
        return std::make_shared<CancelLaunchTask>(input, output, settings);
    }

    if (cmd == "fire_preload" || cmd == "fire") {
        return std::make_shared<FireAndPreloadTask>(input, output, settings, runtime_state);
    }

    if (cmd == "trigger_lock") {
        return std::make_shared<TriggerLockTask>(output);
    }

    if (cmd == "trigger_free") {
        return std::make_shared<TriggerFreeTask>(output);
    }

    if (cmd == "filling_lift_up") {
        return std::make_shared<FillingLiftUpTask>(input, output, settings);
    }

    if (cmd == "filling_lift_down") {
        return std::make_shared<FillingLiftDownTask>(input, output, settings);
    }

    if (cmd == "manual_control" || cmd == "manual-control" || cmd == "manual") {
        return std::make_shared<ManualControlTask>(input, output, settings);
    }

    if (cmd == "carriage_travel" || cmd == "carriage-travel") {
        return make_carriage_travel_task(input, output, settings);
    }

    if (cmd == "chassis_leveling" || cmd == "chassis-leveling") {
        return std::make_shared<ChassisLevelingTask>(input, output);
    }

    return nullptr;
}

} // namespace rmcs_dart_guidance::manager
