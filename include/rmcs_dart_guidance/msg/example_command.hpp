#pragma once

#include <cstdint>

namespace rmcs_dart_guidance::msg {

enum class ExampleCommand : uint8_t {
    IDLE = 0,
    ABORT = 1,
    RUNNING = 2,
};

inline bool is_active(ExampleCommand cmd) {
    return cmd != ExampleCommand::IDLE && cmd != ExampleCommand::ABORT;
}

inline const char* to_string(ExampleCommand cmd) {
    switch (cmd) {
    case ExampleCommand::IDLE: return "IDLE";
    case ExampleCommand::ABORT: return "ABORT";
    case ExampleCommand::RUNNING: return "RUNNING";
    }
    return "UNKNOWN";
}

} // namespace rmcs_dart_guidance::msg
