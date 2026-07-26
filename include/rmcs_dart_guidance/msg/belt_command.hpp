#pragma once

#include <cstdint>

namespace rmcs_dart_guidance::msg {

enum class BeltCommand : uint8_t {
    IDLE = 0,
    ABORT = 1,
    DOWN_SLOW = 2,
    DOWN_FAST = 3,
    DOWN_SLOW_PART = 4,
    UP_SOFT = 5,
    UP_SOFT_PART = 6,
    UP_HARD = 7,
    BRAKE = 8,
    INIT = 9,
};

inline bool is_active(BeltCommand cmd) {
    return cmd != BeltCommand::IDLE && cmd != BeltCommand::ABORT;
}

inline const char* to_string(BeltCommand cmd) {
    switch (cmd) {
    case BeltCommand::IDLE: return "IDLE";
    case BeltCommand::ABORT: return "ABORT";
    case BeltCommand::DOWN_SLOW: return "DOWN_SLOW";
    case BeltCommand::DOWN_FAST: return "DOWN_FAST";
    case BeltCommand::DOWN_SLOW_PART: return "DOWN_SLOW_PART";
    case BeltCommand::UP_SOFT: return "UP_SOFT";
    case BeltCommand::UP_SOFT_PART: return "UP_SOFT_PART";
    case BeltCommand::UP_HARD: return "UP_HARD";
    case BeltCommand::BRAKE: return "BRAKE";
    case BeltCommand::INIT: return "INIT";
    }
    return "UNKNOWN";
}

} // namespace rmcs_dart_guidance::msg
