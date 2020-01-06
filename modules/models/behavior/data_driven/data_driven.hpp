// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_DATA_DRIVEN_DATA_DRIVEN_HPP_
#define MODULES_MODELS_BEHAVIOR_DATA_DRIVEN_DATA_DRIVEN_HPP_

#include "modules/models/behavior/behavior_model.hpp"
#include "modules/world/world.hpp"

namespace modules {
namespace models {
namespace behavior {

using dynamic::Trajectory;
using world::ObservedWorld;
using world::objects::AgentId;

class BehaviorDataDriven : public BehaviorModel {
 public:
  const Trajectory traj_from_dataset;

  explicit BehaviorDataDriven(commons::Params* params,
                              Trajectory traj_from_dataset_for_agent)
      : BehaviorModel(params), traj_from_dataset(traj_from_dataset_for_agent) {}

  virtual ~BehaviorDataDriven() {}

  Trajectory Plan(float delta_time, const ObservedWorld& observed_world);

  virtual std::shared_ptr<BehaviorModel> Clone() const;
};

inline std::shared_ptr<BehaviorModel> BehaviorDataDriven::Clone() const {
  std::shared_ptr<BehaviorDataDriven> model_ptr =
    std::make_shared<BehaviorDataDriven>(*this);
  return std::dynamic_pointer_cast<BehaviorModel>(model_ptr);
}

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_DATA_DRIVEN_DATA_DRIVEN_HPP_
