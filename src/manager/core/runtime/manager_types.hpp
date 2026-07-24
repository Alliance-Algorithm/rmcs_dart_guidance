#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "manager/core/runtime/action.hpp"

namespace rmcs_dart_guidance::manager {

enum class ManagerLifecycleState : uint8_t {
    IDLE = 0,
    RUNNING = 1,
    ERROR = 2,
};

inline const char* to_string(ManagerLifecycleState state) {
    switch (state) {
    case ManagerLifecycleState::IDLE: return "IDLE";
    case ManagerLifecycleState::RUNNING: return "RUNNING";
    case ManagerLifecycleState::ERROR: return "ERROR";
    }
    return "UNKNOWN";
}

struct ManagerRuntimeState {
    uint32_t fire_count{0};
    ManagerLifecycleState lifecycle_state{ManagerLifecycleState::IDLE};
};

struct ManagerQueuedTaskInfo {
    std::string task_name;
    std::string display_name;

    friend bool operator==(const ManagerQueuedTaskInfo&, const ManagerQueuedTaskInfo&) = default;
};

struct ManagerLastErrorInfo {
    std::string task_name;
    std::string action_name;
    ActionFailureReason reason{ActionFailureReason::NONE};
    int64_t timestamp_ms{0};

    friend bool operator==(const ManagerLastErrorInfo&, const ManagerLastErrorInfo&) = default;
};

} // namespace rmcs_dart_guidance::manager
