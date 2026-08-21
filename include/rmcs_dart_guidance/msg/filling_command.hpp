#pragma once

#include <cstdint>

namespace rmcs_dart_guidance::msg {

enum class FillingCommand : uint8_t {
    IDLE = 0,
    ABORT = 1,
    LIFT_UP = 2,
    LIFT_DOWN = 3,
    LIMIT_FREE = 4,
    LIMIT_LOCK = 5,
    LIMIT_PULSE_FILL = 6,
};

inline bool is_active(FillingCommand cmd) {
    return cmd != FillingCommand::IDLE && cmd != FillingCommand::ABORT;
}

inline const char* to_string(FillingCommand cmd) {
    switch (cmd) {
    case FillingCommand::IDLE: return "IDLE";
    case FillingCommand::ABORT: return "ABORT";
    case FillingCommand::LIFT_UP: return "LIFT_UP";
    case FillingCommand::LIFT_DOWN: return "LIFT_DOWN";
    case FillingCommand::LIMIT_FREE: return "LIMIT_FREE";
    case FillingCommand::LIMIT_LOCK: return "LIMIT_LOCK";
    case FillingCommand::LIMIT_PULSE_FILL: return "LIMIT_PULSE_FILL";
    }
    return "UNKNOWN";
}

} // namespace rmcs_dart_guidance::msg
