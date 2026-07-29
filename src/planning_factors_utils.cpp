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

#include "autoware_state_machine/planning_factors_utils.hpp"

#include <cstdint>
#include <queue>
#include <vector>

namespace autoware_state_machine
{

namespace
{

using tier4_external_api_msgs::msg::PlanningFactor;
using tier4_external_api_msgs::msg::PlanningFactorArray;

double factorPrimaryDistanceM(const PlanningFactor & factor)
{
  if (factor.control_points.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  return static_cast<double>(factor.control_points[0].distance);
}

int8_t planningBehaviorStopPriority(const std::string & behavior_name)
{
  if (behavior_name == "surround_obstacle_checker" || behavior_name == "surrounding_obstacle") {
    return 1;
  }
  if (behavior_name == "obstacle_stop" || behavior_name == "route_obstacle") {
    return 2;
  }
  if (behavior_name == "user_defined_detection_area") {
    return 3;
  }
  if (behavior_name == "virtual_traffic_light") {
    return 4;
  }
  if (behavior_name == "stop_sign") {
    return 5;
  }
  return 10;
}

struct FactorStopInfo
{
  std::string behavior_name;
  double distance;
  int8_t priority;
};

}  // namespace

std::pair<std::string, double> selectNearestPlanningFactorBehaviorWithPriority(
  const PlanningFactorArray & msg, double dist_select_max_m)
{
  static constexpr double kNearDist = 1e-3;
  auto compare = [](const FactorStopInfo & a, const FactorStopInfo & b) -> bool {
    if (a.distance < kNearDist && b.distance < kNearDist) {
      return a.priority > b.priority;
    }
    return a.distance > b.distance;
  };
  std::priority_queue<FactorStopInfo, std::vector<FactorStopInfo>, decltype(compare)> que(compare);

  for (const auto & factor : msg.factors) {
    if (factor.behavior_name.empty()) {
      continue;
    }
    const double d = factorPrimaryDistanceM(factor);
    if (!std::isfinite(d) || d >= dist_select_max_m) {
      continue;
    }
    que.push(FactorStopInfo{
      factor.behavior_name, d, planningBehaviorStopPriority(factor.behavior_name)});
  }

  if (que.empty()) {
    return {"", 0.0};
  }
  const FactorStopInfo top = que.top();
  return {top.behavior_name, top.distance};
}

bool isObstacleApproachBehaviorName(const std::string & behavior_name)
{
  return behavior_name == "route_obstacle" || behavior_name == "user_defined_detection_area" ||
         behavior_name == "obstacle_stop";
}

bool isPlanningSelectedDistAheadOfStopThreshold(
  double dist_m, double stop_approach_dist_threshold_m)
{
  return std::isfinite(dist_m) && dist_m > stop_approach_dist_threshold_m;
}

}  // namespace autoware_state_machine
