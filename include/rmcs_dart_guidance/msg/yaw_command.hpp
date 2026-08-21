#pragma once

#include <cstdint>

namespace rmcs_dart_guidance::msg {

enum class YawCommand : uint8_t {
    IDLE = 0,
    ABORT = 1,
    VISION_AIM = 2,
};

inline bool is_active(YawCommand cmd) {
    return cmd != YawCommand::IDLE && cmd != YawCommand::ABORT;
}

inline const char* to_string(YawCommand cmd) {
    switch (cmd) {
    case YawCommand::IDLE: return "IDLE";
    case YawCommand::ABORT: return "ABORT";
    case YawCommand::VISION_AIM: return "VISION_AIM";
    }
    return "UNKNOWN";
}

} // namespace rmcs_dart_guidance::msg
