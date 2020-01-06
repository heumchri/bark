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

  double start_time = observed_world.get_world_time();
  int current_row = 0;
  while (traj_from_dataset(current_row, StateDefinition::TIME_POSITION) <
             start_time &&
         current_row < traj_from_dataset.rows()) {
    current_row++;
  }

  // Trajectory traj = traj_from_dataset(Eigen::seq(current_row, Eigen::last),
  // Eigen::all);
  Trajectory traj = traj_from_dataset.block(
      current_row, 0, traj_from_dataset.rows() - current_row,
      StateDefinition::TIME_POSITION);

  this->set_last_trajectory(traj);
  return traj;
}

}  // namespace models
}  // namespace modules
