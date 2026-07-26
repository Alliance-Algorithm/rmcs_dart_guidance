#pragma once

#include <cstdint>

namespace rmcs_dart_guidance::msg {

enum class FourZChassisCommand : uint8_t {
    IDLE = 0,
    ABORT = 1,
    CALIBRATE_BOTTOM = 2,
    LEVEL_ZERO = 3,
    DOWN = 4,
};

inline bool is_active(FourZChassisCommand cmd) {
    return cmd != FourZChassisCommand::IDLE && cmd != FourZChassisCommand::ABORT;
}

inline const char* to_string(FourZChassisCommand cmd) {
    switch (cmd) {
    case FourZChassisCommand::IDLE: return "IDLE";
    case FourZChassisCommand::ABORT: return "ABORT";
    case FourZChassisCommand::CALIBRATE_BOTTOM: return "CALIBRATE_BOTTOM";
    case FourZChassisCommand::LEVEL_ZERO: return "LEVEL_ZERO";
    case FourZChassisCommand::DOWN: return "DOWN";
    }
    return "UNKNOWN";
}

} // namespace rmcs_dart_guidance::msg
