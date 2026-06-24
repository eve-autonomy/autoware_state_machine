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

#include <gtest/gtest.h>

#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_adapi_v1_msgs/msg/motion_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/msg/turn_indicators.hpp>
#include <autoware_adapi_v1_msgs/msg/vehicle_status.hpp>
#include <autoware_state_machine_msgs/msg/state_machine.hpp>
#include <go_interface_msgs/msg/vehicle_status.hpp>
#include <tier4_external_api_msgs/msg/hazard_status_stamped.hpp>

using LocalizationState = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
using MotionState = autoware_adapi_v1_msgs::msg::MotionState;
using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
using StateMachine = autoware_state_machine_msgs::msg::StateMachine;
using TurnIndicators = autoware_adapi_v1_msgs::msg::TurnIndicators;
using VehicleStatus = autoware_adapi_v1_msgs::msg::VehicleStatus;
using GoVehicleStatus = go_interface_msgs::msg::VehicleStatus;
using HazardStatusStamped = tier4_external_api_msgs::msg::HazardStatusStamped;

class TestableAutowareStateMachine : public autoware_state_machine::AutowareStateMachine
{
public:
  explicit TestableAutowareStateMachine(const rclcpp::NodeOptions & options)
  : AutowareStateMachine(options) {}

  uint16_t getServiceLayerState() const { return current_service_layer_state_; }
  uint8_t getControlLayerState() const { return current_control_layer_state_; }

  void setLocalizationState(uint16_t state)
  {
    localization_state_.state = state;
    updateStateFromTopics();
  }

  void setRouteState(uint16_t state)
  {
    route_state_.state = state;
    updateStateFromTopics();
  }

  void setMotionState(uint16_t state)
  {
    motion_state_.state = state;
    updateStateFromTopics();
  }

  void setOperationMode(bool control_enabled, uint8_t mode = OperationModeState::AUTONOMOUS)
  {
    operation_mode_state_.is_autoware_control_enabled = control_enabled;
    operation_mode_state_.mode = mode;
    updateStateFromTopics();
  }

  void setEmergencyHolding(bool holding)
  {
    emergency_holding_ = holding;
    updateStateFromTopics();
  }

  void setGoInterfaceFlags(bool voice_flg, bool lock_flg)
  {
    go_interface_vehicle_status_.voice_flg = voice_flg;
    go_interface_vehicle_status_.lock_flg = lock_flg;
    updateStateFromTopics();
  }

  void completeWakeupSound()
  {
    is_playing_wakeup_sound_ = false;
    updateStateFromTopics();
  }

  static rclcpp::NodeOptions createTestNodeOptions()
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"stop_dist_to_prohibit_engage", 0.30},
      {"planning_factors_selection_dist_max_m", 10.0},
    });
    return options;
  }
};

class AutowareStateMachineTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<TestableAutowareStateMachine>(
      TestableAutowareStateMachine::createTestNodeOptions());
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<TestableAutowareStateMachine> node_;
};

TEST_F(AutowareStateMachineTest, StartupPublishesCheckNodeAlive)
{
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_CHECK_NODE_ALIVE);
  EXPECT_EQ(node_->getControlLayerState(), StateMachine::MANUAL);
}

TEST_F(AutowareStateMachineTest, DuringWakeupWhenLocalizationNotInitialized)
{
  node_->completeWakeupSound();
  node_->setLocalizationState(LocalizationState::INITIALIZING);
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_DURING_WAKEUP);
}

TEST_F(AutowareStateMachineTest, WaitingEngageInstructionWhenRouteSetAndStopped)
{
  node_->completeWakeupSound();
  node_->setLocalizationState(LocalizationState::INITIALIZED);
  node_->setRouteState(RouteState::SET);
  node_->setMotionState(MotionState::STOPPED);
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION);
}

TEST_F(AutowareStateMachineTest, EmergencyStopWhenHazardHolding)
{
  node_->completeWakeupSound();
  node_->setLocalizationState(LocalizationState::INITIALIZED);
  node_->setRouteState(RouteState::SET);
  node_->setEmergencyHolding(true);
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_EMERGENCY_STOP);
}

TEST_F(AutowareStateMachineTest, WaitingCallPermissionWhenVoiceAndLockSet)
{
  node_->completeWakeupSound();
  node_->setLocalizationState(LocalizationState::INITIALIZED);
  node_->setRouteState(RouteState::SET);
  node_->setMotionState(MotionState::STOPPED);
  node_->setGoInterfaceFlags(true, true);
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_WAITING_CALL_PERMISSION);
}

TEST_F(AutowareStateMachineTest, ControlLayerAutoWhenAutowareControlEnabled)
{
  node_->completeWakeupSound();
  node_->setLocalizationState(LocalizationState::INITIALIZED);
  node_->setRouteState(RouteState::SET);
  node_->setOperationMode(true);
  node_->setMotionState(MotionState::MOVING);
  EXPECT_EQ(node_->getControlLayerState(), StateMachine::AUTO);
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_INFORM_ENGAGE);
}
