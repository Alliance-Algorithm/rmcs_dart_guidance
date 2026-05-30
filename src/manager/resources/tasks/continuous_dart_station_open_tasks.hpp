#pragma once

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/deferred_task_action.hpp"
#include "manager/resources/actions/fire_count_validation_action.hpp"
#include "manager/resources/tasks/fire_and_preload_task.hpp"
#include "manager/resources/tasks/launch_preparation_with_vision_task.hpp"
#include "manager/resources/vision_aim_profile_provider.hpp"

#include <memory>
#include <string>
#include <utility>

namespace rmcs_dart_guidance::manager {

class ContinuousDartStationOpenTaskBase : public Task {
public:
    ContinuousDartStationOpenTaskBase(
        std::string task_name, std::string description, uint32_t expected_fire_count,
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings, const VisionAimProfileProvider& profile_provider,
        ManagerRuntimeState& runtime_state)
        : Task(std::move(task_name), std::move(description)) {
        then(
            std::make_shared<FireCountValidationAction>(
                "validate_fire_count", runtime_state.fire_count, expected_fire_count));

        append_single_shot_cycle(input, output, settings, profile_provider, runtime_state, 1);
        append_single_shot_cycle(input, output, settings, profile_provider, runtime_state, 2);
    }

private:
    void append_single_shot_cycle(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings, const VisionAimProfileProvider& profile_provider,
        ManagerRuntimeState& runtime_state, uint32_t sequence_index) {
        then(
            std::make_shared<DeferredTaskAction>(
                "launch_prepare_with_vision_" + std::to_string(sequence_index),
                [input, output, settings, &profile_provider, &runtime_state]() mutable {
                    return std::make_shared<LaunchPreparationWithVisionTimeoutTolerantTask>(
                        input, output, settings, profile_provider, runtime_state);
                }));

        then(
            std::make_shared<DeferredTaskAction>(
                "fire_preload_" + std::to_string(sequence_index),
                [input, output, settings, &runtime_state]() mutable {
                    return std::make_shared<FireAndPreloadTask>(
                        input, output, settings, runtime_state);
                }));
    }
};

class FirstDartStationOpenTask : public ContinuousDartStationOpenTaskBase {
public:
    FirstDartStationOpenTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings, const VisionAimProfileProvider& profile_provider,
        ManagerRuntimeState& runtime_state)
        : ContinuousDartStationOpenTaskBase(
              "first_dart_station_open_task", "连续发射第一第二发", 0, input, output, settings,
              profile_provider, runtime_state) {}
};

class SecondDartStationOpenTask : public ContinuousDartStationOpenTaskBase {
public:
    SecondDartStationOpenTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings, const VisionAimProfileProvider& profile_provider,
        ManagerRuntimeState& runtime_state)
        : ContinuousDartStationOpenTaskBase(
              "second_dart_station_open_task", "连续发射第三第四发", 2, input, output, settings,
              profile_provider, runtime_state) {}
};

} // namespace rmcs_dart_guidance::manager
