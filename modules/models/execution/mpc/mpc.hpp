// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_EXECUTION_MPC_MPC_HPP_
#define MODULES_MODELS_EXECUTION_MPC_MPC_HPP_

#include <Eigen/Core>
#include <vector>
// #include "ceres/ceres.h"
#include "ceres/problem.h"
#include "ceres/dynamic_autodiff_cost_function.h"
#include "ceres/solver.h"
#include "ceres/loss_function.h"
#include "glog/logging.h"

#include "modules/models/execution/execution_model.hpp"
#include "modules/models/execution/mpc/cost_functor.hpp"
#include "modules/models/execution/mpc/common.hpp"

namespace modules {
namespace models {
namespace execution {

using dynamic::Trajectory;
using dynamic::DynamicModelPtr;
using dynamic::State;
using Eigen::Dynamic;
using Eigen::Matrix;

class ExecutionModelMpc : public ExecutionModel {
 public:
  explicit ExecutionModelMpc(commons::Params *params);

  ~ExecutionModelMpc() {}

  Matrix<double, Dynamic, Dynamic> get_last_weights() {
    return last_weights_;
  }

  Trajectory get_last_desired_states() { return last_desired_states_; }

  void set_last_weights(const Matrix<double, Dynamic, Dynamic> &weights) {
    last_weights_ = weights;
  }

  void set_last_desired_states(const Trajectory &desired_states) {
    last_desired_states_ = desired_states;
  }

  virtual Trajectory Execute(const float &new_world_time,
                             const Trajectory &trajectory,
                             const DynamicModelPtr dynamic_model,
                             const State current_state);

  Trajectory Optimize(std::vector<double*> parameter_block,
                      const Trajectory &discrete_behavior,
                      const Matrix<double, Dynamic, Dynamic> weights_desired_states);

  virtual ExecutionModel* Clone() const;

 private:
  execution::OptimizationSettings optimization_settings_;
  Matrix<double, Dynamic, Dynamic> last_weights_;
  Trajectory last_desired_states_;
};

inline ExecutionModel* ExecutionModelMpc::Clone() const {
  return new ExecutionModelMpc(*this);
}

}  // namespace execution
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_EXECUTION_MPC_MPC_HPP_
