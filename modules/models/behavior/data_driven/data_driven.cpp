// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <algorithm>

#include "modules/models/behavior/data_driven/data_driven.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {

dynamic::Trajectory behavior::BehaviorDataDriven::Plan(
    float delta_time, const world::ObservedWorld& observed_world) {
  using namespace dynamic;

  this->set_last_trajectory(traj_from_dataset);
  return traj_from_dataset;
}

}  // namespace models
}  // namespace modules
