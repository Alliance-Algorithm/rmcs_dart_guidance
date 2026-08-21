#pragma once

#include <cstdint>

namespace rmcs_dart_guidance::msg {

enum class MechanismStatus : uint8_t {
    IDLE = 0,
    BUSY = 1,
    SUCCEEDED = 2,
    FAILED = 3,
    ABORTED = 4,
};

inline const char* to_string(MechanismStatus status) {
    switch (status) {
    case MechanismStatus::IDLE: return "IDLE";
    case MechanismStatus::BUSY: return "BUSY";
    case MechanismStatus::SUCCEEDED: return "SUCCEEDED";
    case MechanismStatus::FAILED: return "FAILED";
    case MechanismStatus::ABORTED: return "ABORTED";
    }
    return "UNKNOWN";
}

} // namespace rmcs_dart_guidance::msg
