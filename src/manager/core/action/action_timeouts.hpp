#pragma once

#include <cstdint>

namespace rmcs_dart_guidance::manager {

// Per-action timeout (control ticks). Must exceed core stub_complete_ticks (default 50).
// Tune per mechanism when real control replaces stubs.

inline constexpr uint64_t kTimeoutBeltDown = 200;
inline constexpr uint64_t kTimeoutBeltDownHold = 200;
inline constexpr uint64_t kTimeoutBeltUpSoft = 200;
inline constexpr uint64_t kTimeoutBeltUpHard = 200;
inline constexpr uint64_t kTimeoutBeltUpStall = 500;

inline constexpr uint64_t kTimeoutTriggerServo = 200;

inline constexpr uint64_t kTimeoutFillingLift = 200;
inline constexpr uint64_t kTimeoutFillingLimitPulse = 300;

} // namespace rmcs_dart_guidance::manager
