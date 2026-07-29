// Copyright 2020 eve autonomy inc. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE_STATE_MACHINE__PLANNING_FACTORS_UTILS_HPP_
#define AUTOWARE_STATE_MACHINE__PLANNING_FACTORS_UTILS_HPP_

#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "tier4_external_api_msgs/msg/planning_factor_array.hpp"

namespace autoware_state_machine
{

std::pair<std::string, double> selectNearestPlanningFactorBehaviorWithPriority(
  const tier4_external_api_msgs::msg::PlanningFactorArray & msg, double dist_select_max_m);

bool isObstacleApproachBehaviorName(const std::string & behavior_name);

bool isPlanningSelectedDistAheadOfStopThreshold(
  double dist_m, double stop_approach_dist_threshold_m);

}  // namespace autoware_state_machine

#endif  // AUTOWARE_STATE_MACHINE__PLANNING_FACTORS_UTILS_HPP_
