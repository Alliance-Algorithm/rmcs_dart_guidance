#pragma once

#include <cstdint>

namespace rmcs_dart_guidance::msg {

enum class ChassisCommand : uint8_t {
    IDLE = 0,
    ABORT = 1,
    ZERO_CALIBRATE = 2,
    LEVEL = 3,
};

inline bool is_active(ChassisCommand cmd) {
    return cmd != ChassisCommand::IDLE && cmd != ChassisCommand::ABORT;
}

inline const char* to_string(ChassisCommand cmd) {
    switch (cmd) {
    case ChassisCommand::IDLE: return "IDLE";
    case ChassisCommand::ABORT: return "ABORT";
    case ChassisCommand::ZERO_CALIBRATE: return "ZERO_CALIBRATE";
    case ChassisCommand::LEVEL: return "LEVEL";
    }
    return "UNKNOWN";
}

} // namespace rmcs_dart_guidance::msg
