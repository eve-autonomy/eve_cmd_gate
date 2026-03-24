// Copyright 2021 eve autonomy inc. All Rights Reserved.
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


#ifndef ENGAGE_SRV_CONVERTER__ENGAGE_SRV_CONVERTER_HPP_
#define ENGAGE_SRV_CONVERTER__ENGAGE_SRV_CONVERTER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "tier4_api_utils/tier4_api_utils.hpp"
#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"
#include "autoware_adapi_v1_msgs/msg/route_state.hpp"
#include "autoware_adapi_v1_msgs/msg/route.hpp"
#include "autoware_adapi_v1_msgs/srv/accept_start.hpp"
#include "autoware_adapi_v1_msgs/msg/motion_state.hpp"
#include "autoware_state_machine_msgs/msg/state_lock.hpp"
#include "autoware_state_machine_msgs/msg/state_sound_done.hpp"
#include "autoware_state_machine_msgs/msg/state_machine.hpp"
#include "tier4_external_api_msgs/srv/engage.hpp"
#include "eve_cmd_gate_msgs/msg/engage_request_state.hpp"
#include "tier4_external_api_msgs/msg/hazard_status_stamped.hpp"

namespace eve_cmd_gate
{

class EveCmdGate : public rclcpp::Node
{
public:
  explicit EveCmdGate(const rclcpp::NodeOptions & options);

private:
  using ExternalEngage = tier4_external_api_msgs::srv::Engage;
  using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
  using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
  using Route = autoware_adapi_v1_msgs::msg::Route;
  using StateLock = autoware_state_machine_msgs::msg::StateLock;
  using StateSoundDone = autoware_state_machine_msgs::msg::StateSoundDone;
  using HazardStatusStamped = tier4_external_api_msgs::msg::HazardStatusStamped;
  using MotionState = autoware_adapi_v1_msgs::msg::MotionState;

  // Callback group
  rclcpp::CallbackGroup::SharedPtr callback_group_service_;
  rclcpp::CallbackGroup::SharedPtr callback_group_subscription_;

  // Subscription
  rclcpp::Subscription<OperationModeState>::SharedPtr sub_operation_mode_state_;
  rclcpp::Subscription<RouteState>::SharedPtr sub_routing_state_;
  rclcpp::Subscription<Route>::SharedPtr sub_routing_route_;
  rclcpp::Subscription<StateLock>::SharedPtr sub_lock_state_;
  rclcpp::Subscription<StateSoundDone>::SharedPtr sub_engage_sound_done_;
  rclcpp::Subscription<HazardStatusStamped>::SharedPtr sub_emergency_holding_;
  rclcpp::Subscription<MotionState>::SharedPtr sub_motion_state_;

  // Publisher
  rclcpp::Publisher<eve_cmd_gate_msgs::msg::EngageRequestState>::SharedPtr pub_state_;

  // Service
  rclcpp::Service<tier4_external_api_msgs::srv::Engage>::SharedPtr srv_engage_;

  // Client
  rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedPtr cli_engage_;
  rclcpp::Client<autoware_adapi_v1_msgs::srv::AcceptStart>::SharedPtr cli_accept_start_;

  // Class Variables
  std::shared_mutex lock_state_mtx_;

  OperationModeState operation_state_;
  uint16_t current_delivery_reservation_state_;
  uint16_t on_sound_done_state_;
  bool on_sound_playing_flg_;
  bool is_emergency_holding_;

  typedef struct tuple
  {
    bool sound_enable;
  } SoundDoneTuple_t;
  std::map<uint16_t, SoundDoneTuple_t> sound_param_;
  bool is_engage_requesting_;
  bool is_engage_accepted_;
  uint16_t routing_state_;
  Route routing_route_;
  uint16_t motion_state_;
  bool sound_done_for_restart_;

  // Callback
  void execEngageProcess(
    const ExternalEngage::Request::SharedPtr request,
    const ExternalEngage::Response::SharedPtr response);
  bool isEmergencyHolding(void);
  bool isRequestReset(void);
  void onOperationModeStatus(const OperationModeState::SharedPtr msg);
  void onRoutingStatus(const RouteState::SharedPtr msg);
  void onRoutingRoute(const Route::SharedPtr msg);
  void onLockState(const StateLock::SharedPtr msg);
  void onStateSoundDone(const StateSoundDone::SharedPtr msg);
  void onHazardStatusStamped(const HazardStatusStamped::SharedPtr msg);
  void onMotionState(const MotionState::SharedPtr msg);
  void tryCallAcceptStart();

  // Class Method
  void setEngageProcess(bool request, bool accept);
  std::pair<bool, bool> getEngageProcess();
  bool isWaitingEngage();
  bool isDriving();
};

}  // namespace engage_srv_converter

#endif  // ENGAGE_SRV_CONVERTER__ENGAGE_SRV_CONVERTER_HPP_
