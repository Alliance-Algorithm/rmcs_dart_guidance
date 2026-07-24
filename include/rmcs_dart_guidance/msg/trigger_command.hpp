#pragma once

#include <cstdint>

namespace rmcs_dart_guidance::msg {

enum class TriggerCommand : uint8_t {
    IDLE = 0,
    ABORT = 1,
    SERVO_LOCK = 2,
    SERVO_FREE = 3,
    SLIDE_UP = 4,
    SLIDE_DOWN = 5,
    SLIDE_STALL_CALIB = 6,
    SLIDE_GOTO = 7,
};

inline bool is_active(TriggerCommand cmd) {
    return cmd != TriggerCommand::IDLE && cmd != TriggerCommand::ABORT;
}

inline const char* to_string(TriggerCommand cmd) {
    switch (cmd) {
    case TriggerCommand::IDLE: return "IDLE";
    case TriggerCommand::ABORT: return "ABORT";
    case TriggerCommand::SERVO_LOCK: return "SERVO_LOCK";
    case TriggerCommand::SERVO_FREE: return "SERVO_FREE";
    case TriggerCommand::SLIDE_UP: return "SLIDE_UP";
    case TriggerCommand::SLIDE_DOWN: return "SLIDE_DOWN";
    case TriggerCommand::SLIDE_STALL_CALIB: return "SLIDE_STALL_CALIB";
    case TriggerCommand::SLIDE_GOTO: return "SLIDE_GOTO";
    }
    return "UNKNOWN";
}

} // namespace rmcs_dart_guidance::msg
