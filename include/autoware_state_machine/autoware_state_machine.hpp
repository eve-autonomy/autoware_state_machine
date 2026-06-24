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
// limitations under the License

#ifndef AUTOWARE_STATE_MACHINE__AUTOWARE_STATE_MACHINE_HPP_
#define AUTOWARE_STATE_MACHINE__AUTOWARE_STATE_MACHINE_HPP_

#include <string>
#include <utility>

#include "autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp"
#include "autoware_adapi_v1_msgs/msg/motion_state.hpp"
#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"
#include "autoware_adapi_v1_msgs/msg/route_state.hpp"
#include "autoware_adapi_v1_msgs/msg/vehicle_status.hpp"
#include "autoware_state_machine_msgs/msg/state_machine.hpp"
#include "autoware_state_machine_msgs/msg/state_sound_done.hpp"
#include "go_interface_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tier4_external_api_msgs/msg/hazard_status_stamped.hpp"
#include "tier4_external_api_msgs/msg/planning_factor_array.hpp"

namespace autoware_state_machine
{

class AutowareStateMachine : public rclcpp::Node
{
public:
  explicit AutowareStateMachine(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~AutowareStateMachine() override;

protected:
  void updateStateFromTopics();
  void publishStateIfChanged(uint16_t service_layer_state, uint8_t control_layer_state);
  void setSoundPlayingFlagForState(uint16_t service_layer_state);

  // ADAPI / external topic state
  autoware_adapi_v1_msgs::msg::MotionState motion_state_;
  autoware_adapi_v1_msgs::msg::MotionState prev_motion_state_;
  autoware_adapi_v1_msgs::msg::RouteState route_state_;
  autoware_adapi_v1_msgs::msg::RouteState prev_route_state_;
  autoware_adapi_v1_msgs::msg::LocalizationInitializationState localization_state_;
  autoware_adapi_v1_msgs::msg::LocalizationInitializationState prev_localization_state_;
  autoware_adapi_v1_msgs::msg::OperationModeState operation_mode_state_;
  autoware_adapi_v1_msgs::msg::OperationModeState prev_operation_mode_state_;
  autoware_adapi_v1_msgs::msg::VehicleStatus adapi_vehicle_status_;
  autoware_adapi_v1_msgs::msg::VehicleStatus prev_adapi_vehicle_status_;
  go_interface_msgs::msg::VehicleStatus go_interface_vehicle_status_;
  go_interface_msgs::msg::VehicleStatus prev_go_interface_vehicle_status_;
  bool emergency_holding_{false};

  // Session flags
  bool has_started_driving_{false};
  bool driving_session_had_moving_{false};
  bool post_engage_sound_latched_{false};
  bool pending_autonomous_control_inform_engage_{false};

  // Sound playback flags (state_sound_done clears these)
  bool is_playing_wakeup_sound_{true};
  bool is_playing_arrival_sound_{false};
  bool is_playing_engage_sound_{false};
  bool is_playing_restart_sound_{false};

  // Planning factors cache
  std::pair<std::string, double> cached_planning_selected_nearest_{"", 0.0};
  bool planning_selected_stop_reason_initialized_{false};

  uint16_t current_service_layer_state_{
    autoware_state_machine_msgs::msg::StateMachine::STATE_UNDEFINED};
  uint8_t current_control_layer_state_{autoware_state_machine_msgs::msg::StateMachine::MANUAL};

  double stop_approach_dist_threshold_m_{0.35};
  double planning_factors_selection_dist_max_m_{10.0};

private:
  void callbackMotionState(const autoware_adapi_v1_msgs::msg::MotionState::ConstSharedPtr msg);
  void callbackRouteState(const autoware_adapi_v1_msgs::msg::RouteState::ConstSharedPtr msg);
  void callbackLocalizationState(
    const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::ConstSharedPtr msg);
  void callbackOperationModeState(
    const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg);
  void callbackAdapiVehicleStatus(
    const autoware_adapi_v1_msgs::msg::VehicleStatus::ConstSharedPtr msg);
  void callbackGoInterfaceVehicleStatus(
    const go_interface_msgs::msg::VehicleStatus::ConstSharedPtr msg);
  void callbackHazardStatus(
    const tier4_external_api_msgs::msg::HazardStatusStamped::ConstSharedPtr msg);
  void callbackPlanningFactors(
    const tier4_external_api_msgs::msg::PlanningFactorArray::ConstSharedPtr msg);
  void callbackStateSoundDone(
    const autoware_state_machine_msgs::msg::StateSoundDone::ConstSharedPtr msg);

  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::MotionState>::SharedPtr sub_motion_state_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr sub_route_state_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>::SharedPtr
    sub_localization_state_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr
    sub_operation_mode_state_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::VehicleStatus>::SharedPtr
    sub_adapi_vehicle_status_;
  rclcpp::Subscription<go_interface_msgs::msg::VehicleStatus>::SharedPtr
    sub_go_interface_vehicle_status_;
  rclcpp::Subscription<tier4_external_api_msgs::msg::HazardStatusStamped>::SharedPtr
    sub_hazard_status_;
  rclcpp::Subscription<tier4_external_api_msgs::msg::PlanningFactorArray>::SharedPtr
    sub_planning_factors_;
  rclcpp::Subscription<autoware_state_machine_msgs::msg::StateSoundDone>::SharedPtr
    sub_state_sound_done_;

  rclcpp::Publisher<autoware_state_machine_msgs::msg::StateMachine>::SharedPtr pub_state_;
};

}  // namespace autoware_state_machine

#endif  // AUTOWARE_STATE_MACHINE__AUTOWARE_STATE_MACHINE_HPP_
