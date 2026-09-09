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

#include "autoware_state_machine/autoware_state_machine.hpp"

#include <utility>

#include "autoware_state_machine/planning_factors_utils.hpp"

namespace autoware_state_machine
{

AutowareStateMachine::AutowareStateMachine(const rclcpp::NodeOptions & options)
: Node("autoware_state_machine", options)
{
  const double stop_dist_to_prohibit_engage =
    this->declare_parameter<double>("stop_dist_to_prohibit_engage", 0.30);
  stop_approach_dist_threshold_m_ = stop_dist_to_prohibit_engage + 0.05;
  planning_factors_selection_dist_max_m_ =
    this->declare_parameter<double>("planning_factors_selection_dist_max_m", 10.0);

  const auto qos_transient_local = rclcpp::QoS{1}.transient_local();

  sub_motion_state_ = this->create_subscription<autoware_adapi_v1_msgs::msg::MotionState>(
    "/api/motion/state", rclcpp::QoS{1},
    std::bind(&AutowareStateMachine::callbackMotionState, this, std::placeholders::_1));
  sub_route_state_ = this->create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
    "/api/routing/state", rclcpp::QoS{1},
    std::bind(&AutowareStateMachine::callbackRouteState, this, std::placeholders::_1));
  sub_localization_state_ =
    this->create_subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>(
    "/api/localization/initialization_state", rclcpp::QoS{1},
    std::bind(&AutowareStateMachine::callbackLocalizationState, this, std::placeholders::_1));
  sub_operation_mode_state_ =
    this->create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
    "/api/operation_mode/state", rclcpp::QoS{1},
    std::bind(&AutowareStateMachine::callbackOperationModeState, this, std::placeholders::_1));
  sub_adapi_vehicle_status_ = this->create_subscription<autoware_adapi_v1_msgs::msg::VehicleStatus>(
    "/api/vehicle/status", rclcpp::SensorDataQoS(),
    std::bind(&AutowareStateMachine::callbackAdapiVehicleStatus, this, std::placeholders::_1));
  sub_go_interface_vehicle_status_ = this->create_subscription<go_interface_msgs::msg::VehicleStatus>(
    "api_vehicle_status", rclcpp::QoS{1},
    std::bind(
      &AutowareStateMachine::callbackGoInterfaceVehicleStatus, this, std::placeholders::_1));
  sub_hazard_status_ =
    this->create_subscription<tier4_external_api_msgs::msg::HazardStatusStamped>(
    "/api/external/get/hazard_status", rclcpp::QoS{1},
    std::bind(&AutowareStateMachine::callbackHazardStatus, this, std::placeholders::_1));
  sub_planning_factors_ =
    this->create_subscription<tier4_external_api_msgs::msg::PlanningFactorArray>(
    "/api/external/get/planning_factors", rclcpp::QoS{1},
    std::bind(&AutowareStateMachine::callbackPlanningFactors, this, std::placeholders::_1));
  sub_state_sound_done_ =
    this->create_subscription<autoware_state_machine_msgs::msg::StateSoundDone>(
    "/autoware_state_machine/state_sound_done", qos_transient_local,
    std::bind(&AutowareStateMachine::callbackStateSoundDone, this, std::placeholders::_1));

  pub_state_ = this->create_publisher<autoware_state_machine_msgs::msg::StateMachine>(
    "/autoware_state_machine/state", rclcpp::QoS{3}.transient_local());

  updateStateFromTopics();
}

AutowareStateMachine::~AutowareStateMachine() = default;

void AutowareStateMachine::setSoundPlayingFlagForState(const uint16_t service_layer_state)
{
  using StateMachine = autoware_state_machine_msgs::msg::StateMachine;
  if (service_layer_state == StateMachine::STATE_CHECK_NODE_ALIVE) {
    is_playing_wakeup_sound_ = true;
  } else if (service_layer_state == StateMachine::STATE_INFORM_ENGAGE) {
    is_playing_engage_sound_ = true;
  } else if (service_layer_state == StateMachine::STATE_INFORM_RESTART) {
    is_playing_restart_sound_ = true;
  } else if (service_layer_state == StateMachine::STATE_ARRIVED_GOAL) {
    is_playing_arrival_sound_ = true;
  }
}

void AutowareStateMachine::publishStateIfChanged(
  const uint16_t service_layer_state, const uint8_t control_layer_state)
{
  if (service_layer_state == current_service_layer_state_ &&
    control_layer_state == current_control_layer_state_)
  {
    return;
  }

  current_service_layer_state_ = service_layer_state;
  current_control_layer_state_ = control_layer_state;
  setSoundPlayingFlagForState(service_layer_state);

  autoware_state_machine_msgs::msg::StateMachine pub;
  pub.stamp = this->now();
  pub.service_layer_state = service_layer_state;
  pub.control_layer_state = control_layer_state;
  pub_state_->publish(pub);

  RCLCPP_INFO(
    this->get_logger(),
    "[autoware_state_machine] service_layer_state=%u control_layer_state=%u",
    service_layer_state, control_layer_state);
}

void AutowareStateMachine::updateStateFromTopics()
{
  using StateMachine = autoware_state_machine_msgs::msg::StateMachine;
  using MotionState = autoware_adapi_v1_msgs::msg::MotionState;
  using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
  using LocalizationState = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
  using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
  using TurnIndicators = autoware_adapi_v1_msgs::msg::TurnIndicators;

  if (emergency_holding_ ||
    route_state_.state != RouteState::SET ||
    localization_state_.state != LocalizationState::INITIALIZED)
  {
    stop_before_first_move_ = false;
    has_started_driving_ = false;
    driving_session_had_moving_ = false;
  }
  if (emergency_holding_ ||
    localization_state_.state != LocalizationState::INITIALIZED)
  {
    pending_autonomous_control_inform_engage_ = false;
  }

  uint8_t control_layer_state = StateMachine::MANUAL;
  if (operation_mode_state_.is_autoware_control_enabled) {
    control_layer_state = StateMachine::AUTO;
  }

  uint16_t service_layer_state = StateMachine::STATE_UNDEFINED;

  const std::string & planning_sel_name = cached_planning_selected_nearest_.first;
  const double planning_sel_dist = cached_planning_selected_nearest_.second;
  const bool planning_sel_dist_ahead =
    isPlanningSelectedDistAheadOfStopThreshold(planning_sel_dist, stop_approach_dist_threshold_m_);
  const bool planning_p14_stop_factor =
    isObstacleApproachBehaviorName(planning_sel_name) ||
    ((planning_sel_name == "surround_obstacle_checker" ||
      planning_sel_name == "surrounding_obstacle") &&
    post_engage_sound_latched_);

  if (is_playing_wakeup_sound_) {
    service_layer_state = StateMachine::STATE_CHECK_NODE_ALIVE;
  } else if (localization_state_.state != LocalizationState::INITIALIZED) {
    service_layer_state = StateMachine::STATE_DURING_WAKEUP;
  } else if (emergency_holding_) {
    service_layer_state = StateMachine::STATE_EMERGENCY_STOP;
  } else if (is_playing_arrival_sound_) {
    service_layer_state = StateMachine::STATE_ARRIVED_GOAL;
  } else if (is_playing_engage_sound_) {
    service_layer_state = StateMachine::STATE_INFORM_ENGAGE;
  } else if (is_playing_restart_sound_) {
    service_layer_state = StateMachine::STATE_INFORM_RESTART;
  } else if (route_state_.state == RouteState::ARRIVED) {
    service_layer_state = StateMachine::STATE_ARRIVED_GOAL;
  } else if (route_state_.state == RouteState::UNSET ||
    route_state_.state == RouteState::UNKNOWN ||
    route_state_.state == RouteState::CHANGING)
  {
    service_layer_state = StateMachine::STATE_DURING_RECEIVE_ROUTE;
  } else if (route_state_.state == RouteState::SET &&
    motion_state_.state == MotionState::STOPPED &&
    !has_started_driving_ &&
    go_interface_vehicle_status_.voice_flg &&
    go_interface_vehicle_status_.lock_flg)
  {
    service_layer_state = StateMachine::STATE_WAITING_CALL_PERMISSION;
  } else if (pending_autonomous_control_inform_engage_ &&
    route_state_.state == RouteState::SET &&
    motion_state_.state == MotionState::STOPPED &&
    operation_mode_state_.is_autoware_control_enabled &&
    operation_mode_state_.mode == OperationModeState::AUTONOMOUS &&
    !has_started_driving_ &&
    !is_playing_engage_sound_)
  {
    service_layer_state = StateMachine::STATE_INFORM_ENGAGE;
    pending_autonomous_control_inform_engage_ = false;
  } else if (route_state_.state == RouteState::SET &&
    motion_state_.state == MotionState::STOPPED &&
    !has_started_driving_)
  {
    service_layer_state = StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION;
  } else if (motion_state_.state == MotionState::STOPPED &&
    has_started_driving_ &&
    route_state_.state == RouteState::SET &&
    (driving_session_had_moving_ || planning_p14_stop_factor))
  {
    if (!driving_session_had_moving_) {
      stop_before_first_move_ = true;
    }
    const std::string & bname = planning_sel_name;
    if ((bname == "surround_obstacle_checker" || bname == "surrounding_obstacle") &&
      post_engage_sound_latched_)
    {
      service_layer_state = StateMachine::STATE_STOP_DUETO_SURROUNDING_PROXIMITY;
    } else if (isObstacleApproachBehaviorName(bname)) {
      service_layer_state = StateMachine::STATE_STOP_DUETO_APPROACHING_OBSTACLE;
    } else {
      service_layer_state = StateMachine::STATE_STOP_DUETO_TRAFFIC_CONDITION;
    }
  } else if (has_started_driving_ &&
    route_state_.state == RouteState::SET &&
    motion_state_.state == MotionState::STOPPED &&
    !driving_session_had_moving_)
  {
    service_layer_state = StateMachine::STATE_INSTRUCT_ENGAGE;
  } else if (motion_state_.state == MotionState::STARTING && !has_started_driving_) {
    service_layer_state = StateMachine::STATE_INFORM_ENGAGE;
  } else if (motion_state_.state == MotionState::STARTING && has_started_driving_ && !driving_session_had_moving_) {
    service_layer_state = stop_before_first_move_ ? StateMachine::STATE_INFORM_RESTART : StateMachine::STATE_INSTRUCT_ENGAGE;
  } else if (motion_state_.state == MotionState::STARTING && has_started_driving_ && driving_session_had_moving_) {
    service_layer_state = StateMachine::STATE_INFORM_RESTART;
  } else if (
    motion_state_.state == MotionState::MOVING &&
    route_state_.state == RouteState::SET &&
    localization_state_.state == LocalizationState::INITIALIZED &&
    operation_mode_state_.is_autoware_control_enabled &&
    !has_started_driving_ &&
    !is_playing_engage_sound_)
  {
    service_layer_state = StateMachine::STATE_INFORM_ENGAGE;
  } else if (motion_state_.state == MotionState::MOVING &&
    localization_state_.state == LocalizationState::INITIALIZED &&
    operation_mode_state_.is_autoware_control_enabled &&
    route_state_.state == RouteState::SET)
  {
    driving_session_had_moving_ = true;
    stop_before_first_move_ = false;

    const std::string & bname = planning_sel_name;
    if ((bname == "surround_obstacle_checker" || bname == "surrounding_obstacle") &&
      post_engage_sound_latched_)
    {
      service_layer_state = StateMachine::STATE_STOP_DUETO_SURROUNDING_PROXIMITY;
    } else if (planning_sel_dist_ahead && isObstacleApproachBehaviorName(bname)) {
      service_layer_state = StateMachine::STATE_RUNNING_TOWARD_OBSTACLE;
    } else if (planning_sel_dist_ahead && bname == "virtual_traffic_light") {
      service_layer_state = StateMachine::STATE_RUNNING_TOWARD_STOP_LINE;
    } else if (adapi_vehicle_status_.turn_indicators.status == TurnIndicators::LEFT) {
      service_layer_state = StateMachine::STATE_TURNING_LEFT;
    } else if (adapi_vehicle_status_.turn_indicators.status == TurnIndicators::RIGHT) {
      service_layer_state = StateMachine::STATE_TURNING_RIGHT;
    } else {
      service_layer_state = StateMachine::STATE_RUNNING;
    }
  } else {
    service_layer_state = StateMachine::STATE_UNDEFINED;
  }

  publishStateIfChanged(service_layer_state, control_layer_state);
}

void AutowareStateMachine::callbackMotionState(
  const autoware_adapi_v1_msgs::msg::MotionState::ConstSharedPtr msg)
{
  using MotionState = autoware_adapi_v1_msgs::msg::MotionState;
  using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
  using LocalizationState = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
  using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;

  prev_motion_state_ = motion_state_;
  motion_state_ = *msg;

  if (prev_motion_state_.state == MotionState::STOPPED &&
    motion_state_.state == MotionState::MOVING &&
    has_started_driving_ &&
    driving_session_had_moving_ &&
    !is_playing_restart_sound_)
  {
    is_playing_restart_sound_ = true;
  }

  if (prev_motion_state_.state == MotionState::MOVING &&
    motion_state_.state == MotionState::STOPPED &&
    route_state_.state != RouteState::SET &&
    operation_mode_state_.is_autoware_control_enabled &&
    operation_mode_state_.mode == OperationModeState::AUTONOMOUS &&
    localization_state_.state == LocalizationState::INITIALIZED &&
    !emergency_holding_ &&
    !has_started_driving_ &&
    !is_playing_engage_sound_)
  {
    pending_autonomous_control_inform_engage_ = true;
  }

  if (prev_motion_state_.state != motion_state_.state) {
    updateStateFromTopics();
  }
}

void AutowareStateMachine::callbackRouteState(
  const autoware_adapi_v1_msgs::msg::RouteState::ConstSharedPtr msg)
{
  using RouteState = autoware_adapi_v1_msgs::msg::RouteState;

  prev_route_state_ = route_state_;
  route_state_ = *msg;

  if (route_state_.state == RouteState::UNSET || route_state_.state == RouteState::UNKNOWN) {
    post_engage_sound_latched_ = false;
    pending_autonomous_control_inform_engage_ = false;
    planning_selected_stop_reason_initialized_ = false;
    cached_planning_selected_nearest_ = {"", 0.0};
    driving_session_had_moving_ = false;
    stop_before_first_move_ = false;
  }

  if (prev_route_state_.state != route_state_.state) {
    updateStateFromTopics();
  }
}

void AutowareStateMachine::callbackLocalizationState(
  const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::ConstSharedPtr msg)
{
  prev_localization_state_ = localization_state_;
  localization_state_ = *msg;

  if (prev_localization_state_.state != localization_state_.state) {
    updateStateFromTopics();
  }
}

void AutowareStateMachine::callbackOperationModeState(
  const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg)
{
  using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
  using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
  using MotionState = autoware_adapi_v1_msgs::msg::MotionState;

  prev_operation_mode_state_ = operation_mode_state_;
  operation_mode_state_ = *msg;

  if (prev_operation_mode_state_.mode != operation_mode_state_.mode ||
    prev_operation_mode_state_.is_autoware_control_enabled !=
    operation_mode_state_.is_autoware_control_enabled)
  {
    const bool control_rising =
      !prev_operation_mode_state_.is_autoware_control_enabled &&
      operation_mode_state_.is_autoware_control_enabled;
    const bool mode_became_autonomous =
      prev_operation_mode_state_.mode != OperationModeState::AUTONOMOUS &&
      operation_mode_state_.mode == OperationModeState::AUTONOMOUS;
    if ((control_rising || mode_became_autonomous) &&
      operation_mode_state_.is_autoware_control_enabled &&
      operation_mode_state_.mode == OperationModeState::AUTONOMOUS &&
      route_state_.state == RouteState::SET &&
      motion_state_.state == MotionState::STOPPED &&
      !has_started_driving_)
    {
      pending_autonomous_control_inform_engage_ = true;
    }

    updateStateFromTopics();
  }
}

void AutowareStateMachine::callbackAdapiVehicleStatus(
  const autoware_adapi_v1_msgs::msg::VehicleStatus::ConstSharedPtr msg)
{
  prev_adapi_vehicle_status_ = adapi_vehicle_status_;
  adapi_vehicle_status_ = *msg;

  if (prev_adapi_vehicle_status_.turn_indicators.status !=
    adapi_vehicle_status_.turn_indicators.status)
  {
    updateStateFromTopics();
  }
}

void AutowareStateMachine::callbackGoInterfaceVehicleStatus(
  const go_interface_msgs::msg::VehicleStatus::ConstSharedPtr msg)
{
  prev_go_interface_vehicle_status_ = go_interface_vehicle_status_;
  go_interface_vehicle_status_ = *msg;

  if (prev_go_interface_vehicle_status_.voice_flg != go_interface_vehicle_status_.voice_flg ||
    prev_go_interface_vehicle_status_.lock_flg != go_interface_vehicle_status_.lock_flg)
  {
    updateStateFromTopics();
  }
}

void AutowareStateMachine::callbackHazardStatus(
  const tier4_external_api_msgs::msg::HazardStatusStamped::ConstSharedPtr msg)
{
  const bool prev_emergency_holding = emergency_holding_;
  emergency_holding_ = msg->status.emergency_holding;

  if (prev_emergency_holding != emergency_holding_) {
    updateStateFromTopics();
  }
}

void AutowareStateMachine::callbackPlanningFactors(
  const tier4_external_api_msgs::msg::PlanningFactorArray::ConstSharedPtr msg)
{
  const std::pair<std::string, double> nearest = selectNearestPlanningFactorBehaviorWithPriority(
    *msg, planning_factors_selection_dist_max_m_);
  const bool dist_ahead_curr =
    isPlanningSelectedDistAheadOfStopThreshold(nearest.second, stop_approach_dist_threshold_m_);
  const bool dist_ahead_cached = isPlanningSelectedDistAheadOfStopThreshold(
    cached_planning_selected_nearest_.second, stop_approach_dist_threshold_m_);

  if (planning_selected_stop_reason_initialized_ &&
    nearest.first == cached_planning_selected_nearest_.first &&
    dist_ahead_curr == dist_ahead_cached)
  {
    return;
  }

  cached_planning_selected_nearest_ = nearest;
  planning_selected_stop_reason_initialized_ = true;
  updateStateFromTopics();
}

void AutowareStateMachine::callbackStateSoundDone(
  const autoware_state_machine_msgs::msg::StateSoundDone::ConstSharedPtr msg)
{
  using StateMachine = autoware_state_machine_msgs::msg::StateMachine;

  if (!msg->done) {
    return;
  }
  if (current_service_layer_state_ != msg->state) {
    return;
  }

  if (msg->state == StateMachine::STATE_CHECK_NODE_ALIVE) {
    is_playing_wakeup_sound_ = false;
  } else if (msg->state == StateMachine::STATE_INFORM_ENGAGE) {
    is_playing_engage_sound_ = false;
    post_engage_sound_latched_ = true;
    has_started_driving_ = true;
  } else if (msg->state == StateMachine::STATE_INFORM_RESTART) {
    is_playing_restart_sound_ = false;
    has_started_driving_ = true;
  } else if (msg->state == StateMachine::STATE_ARRIVED_GOAL) {
    is_playing_arrival_sound_ = false;
  }

  updateStateFromTopics();
}

}  // namespace autoware_state_machine

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(autoware_state_machine::AutowareStateMachine)
