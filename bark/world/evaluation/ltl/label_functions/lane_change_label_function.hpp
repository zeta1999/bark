// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_LTL_LABELS_LANE_CHANGE_LABEL_FUNCTION_HPP_
#define MODULES_WORLD_EVALUATION_LTL_LABELS_LANE_CHANGE_LABEL_FUNCTION_HPP_

#include <string>
#include <vector>

#include "bark/world/evaluation/ltl/label_functions/base_label_function.hpp"
#include "bark/world/objects/agent.hpp"
#include "bark/world/objects/object.hpp"

namespace modules {
namespace world {
namespace evaluation {

using modules::commons::transformation::FrenetPosition;
using modules::world::objects::AgentPtr;

class LaneChangeLabelFunction : public BaseLabelFunction {
 public:
  using BaseLabelFunction::BaseLabelFunction;
  LabelMap Evaluate(const world::ObservedWorld& observed_world) const override;
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_LTL_LABELS_LANE_CHANGE_LABEL_FUNCTION_HPP_