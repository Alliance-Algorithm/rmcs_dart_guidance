#pragma once

#include <cstdint>

namespace rmcs_dart_guidance::msg {

enum class BeltCommand : uint8_t {
    IDLE = 0,
    ABORT = 1,
    DOWN_SLOW = 2,
    DOWN_FAST = 3,
    DOWN_HOLD = 4,
    UP_SOFT = 5,
    UP_HARD = 6,
    UP_STALL = 7,
    BRAKE = 8,
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
    case BeltCommand::DOWN_HOLD: return "DOWN_HOLD";
    case BeltCommand::UP_SOFT: return "UP_SOFT";
    case BeltCommand::UP_HARD: return "UP_HARD";
    case BeltCommand::UP_STALL: return "UP_STALL";
    case BeltCommand::BRAKE: return "BRAKE";
    }
    return "UNKNOWN";
}

} // namespace rmcs_dart_guidance::msg
