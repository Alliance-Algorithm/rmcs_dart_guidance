#pragma once

#include <cstdint>

namespace rmcs_dart_guidance::msg {

enum class TriggerCommand : uint8_t {
    IDLE = 0,
    ABORT = 1,
    TRIGGER_FREE = 2,
    TRIGGER_LOCK = 3,
    CARRIAGE_UP = 4,
    CARRIAGE_DOWN = 5,
    CARRIAGE_CALIBRATE = 6,
    CARRIAGE_GOTO = 7,
};

inline bool is_active(TriggerCommand cmd) {
    return cmd != TriggerCommand::IDLE && cmd != TriggerCommand::ABORT;
}

inline const char* to_string(TriggerCommand cmd) {
    switch (cmd) {
    case TriggerCommand::IDLE: return "IDLE";
    case TriggerCommand::ABORT: return "ABORT";
    case TriggerCommand::TRIGGER_FREE: return "TRIGGER_FREE";
    case TriggerCommand::TRIGGER_LOCK: return "TRIGGER_LOCK";
    case TriggerCommand::CARRIAGE_UP: return "CARRIAGE_UP";
    case TriggerCommand::CARRIAGE_DOWN: return "CARRIAGE_DOWN";
    case TriggerCommand::CARRIAGE_CALIBRATE: return "CARRIAGE_CALIBRATE";
    case TriggerCommand::CARRIAGE_GOTO: return "CARRIAGE_GOTO";
    }
    return "UNKNOWN";
}

} // namespace rmcs_dart_guidance::msg