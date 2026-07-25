#pragma once

#include <rmcs_dart_guidance/resource/belt_resource.hpp>
#include <rmcs_dart_guidance/resource/filling_resource.hpp>
#include <rmcs_dart_guidance/resource/trigger_resource.hpp>

namespace rmcs_dart_guidance::manager {

struct MechanismResources {
    BeltResource& belt;
    TriggerResource& trigger;
    FillingResource& filling;
};

} // namespace rmcs_dart_guidance::manager
