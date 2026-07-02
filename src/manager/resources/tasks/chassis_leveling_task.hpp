#pragma once

#include "manager/core/runtime/action_sequence.hpp"
#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/chassis_leveling_action.hpp"

#include <memory>
#include <string>

namespace rmcs_dart_guidance::manager {

class ChassisLevelingTask : public Task {
public:
    ChassisLevelingTask(const ManagerInputContext& input, ManagerOutputContext& output)
        : Task("chassis_leveling_", "底盘调平") {
        auto make_leveling_round = [&](const int round_index) {
            auto round = std::make_shared<ActionSequence>(
                "roll_pitch_leveling_" + std::to_string(round_index));
            round->then(
                std::make_shared<RollLevelingAction>(
                    "roll_leveling_" + std::to_string(round_index),  //
                    output.chassis_leveling_phase,                   //
                    input.leveling_front_left_velocity,              //
                    input.leveling_rear_left_velocity,               //
                    input.leveling_front_left_torque,                //
                    input.leveling_rear_left_torque,                 //
                    input.roll_angle                                 //
                    ));
            round->then(
                std::make_shared<PitchLevelingAction>(
                    "pitch_leveling_" + std::to_string(round_index), //
                    output.chassis_leveling_phase,                   //
                    input.leveling_front_left_velocity,              //
                    input.leveling_front_right_velocity,             //
                    input.leveling_front_left_torque,                //
                    input.leveling_front_right_torque,               //
                    input.leveling_pitch_angle                       //
                    ));
            return round;
        };

        then(make_leveling_round(1));
        then(make_leveling_round(2));
        then(make_leveling_round(3));
    }
};
} // namespace rmcs_dart_guidance::manager
