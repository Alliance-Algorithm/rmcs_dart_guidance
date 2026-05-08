#pragma once

#include "rmcs_utility/power_cycle_state.hpp"

#include <string>
#include <string_view>

namespace rmcs_dart_guidance::manager {

inline constexpr std::string_view kCarriagePowerCycleOriginStateName =
    "dart_carriage_power_cycle_origin";

inline rmcs_utility::PowerCycleDoubleState load_carriage_power_cycle_origin() {
    return rmcs_utility::load_power_cycle_double(kCarriagePowerCycleOriginStateName);
}

inline bool store_carriage_power_cycle_origin(double angle, std::string* error_message = nullptr) {
    return rmcs_utility::store_power_cycle_double(
        kCarriagePowerCycleOriginStateName, angle, error_message);
}

} // namespace rmcs_dart_guidance::manager
