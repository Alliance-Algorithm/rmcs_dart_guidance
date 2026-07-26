#pragma once

#include <cstdint>

namespace rmcs_dart_guidance::msg {

enum class FourDofCommand : uint8_t {
    IDLE = 0,
    ABORT = 1,
    CALIBRATE_BOTTOM = 2,
    LEVEL_ZERO = 3,
    DOWN = 4,
};

inline bool is_active(FourDofCommand cmd) {
    return cmd != FourDofCommand::IDLE && cmd != FourDofCommand::ABORT;
}

inline const char* to_string(FourDofCommand cmd) {
    switch (cmd) {
    case FourDofCommand::IDLE: return "IDLE";
    case FourDofCommand::ABORT: return "ABORT";
    case FourDofCommand::CALIBRATE_BOTTOM: return "CALIBRATE_BOTTOM";
    case FourDofCommand::LEVEL_ZERO: return "LEVEL_ZERO";
    case FourDofCommand::DOWN: return "DOWN";
    }
    return "UNKNOWN";
}

} // namespace rmcs_dart_guidance::msg
