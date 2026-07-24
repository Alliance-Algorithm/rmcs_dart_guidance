#pragma once

#include "manager/resources/belt_resource.hpp"
#include "manager/resources/filling_resource.hpp"
#include "manager/resources/trigger_resource.hpp"

namespace rmcs_dart_guidance::manager {

struct MechanismResources {
    BeltResource& belt;
    TriggerResource& trigger;
    FillingResource& filling;
};

} // namespace rmcs_dart_guidance::manager
