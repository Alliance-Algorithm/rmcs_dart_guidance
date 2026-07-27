#pragma once

#include <rmcs_dart_guidance/resource/belt_resource.hpp>
#include <rmcs_dart_guidance/resource/chassis_resource.hpp>
#include <rmcs_dart_guidance/resource/filling_resource.hpp>
#include <rmcs_dart_guidance/resource/trigger_resource.hpp>

namespace rmcs_dart_guidance::manager {

struct MechanismResources {
    BeltResource& belt;
    TriggerResource& trigger;
    FillingResource& filling;
    ChassisResource& chassis;
};

} // namespace rmcs_dart_guidance::manager
