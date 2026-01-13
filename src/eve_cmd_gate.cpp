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


#include <memory>
#include "eve_cmd_gate/eve_cmd_gate.hpp"

namespace eve_cmd_gate
{

#define DEBUG_THROTTLE_TIME 5000  // ms

EveCmdGate::EveCmdGate(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("eve_cmd_gate", options)
{
  using namespace std::placeholders;

  // Callback group
  callback_group_service_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_subscription_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto subscribe_option = rclcpp::SubscriptionOptions();
  subscribe_option.callback_group = callback_group_subscription_;

  // Subscription
  sub_operation_mode_state_ = this->create_subscription<OperationModeState>(
    "/api/operation_mode/state",
    rclcpp::QoS{1}.transient_local(),
    std::bind(&EveCmdGate::onOperationModeStatus, this, _1),
    subscribe_option);

  sub_routing_state_ = this->create_subscription<RouteState>(
    "/api/routing/state",
    rclcpp::QoS{1}.transient_local(),
    std::bind(&EveCmdGate::onRoutingStatus, this, _1),
    subscribe_option);

  sub_routing_route_ = this->create_subscription<Route>(
    "/api/routing/route",
    rclcpp::QoS{1}.transient_local(),
    std::bind(&EveCmdGate::onRoutingRoute, this, _1),
    subscribe_option);

  sub_lock_state_ = this->create_subscription<StateLock>(
    "/go_interface/lock_state",
    rclcpp::QoS{1}.transient_local(),
    std::bind(&EveCmdGate::onLockState, this, _1),
    subscribe_option);

  sub_engage_sound_done_ = this->create_subscription<StateSoundDone>(
    "/autoware_state_machine/state_sound_done",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&EveCmdGate::onStateSoundDone, this, _1),
    subscribe_option);

  sub_emergency_holding_ = this->create_subscription<HazardStatusStamped>(
    "/system/emergency/hazard_status",
    rclcpp::QoS{1},
    std::bind(&EveCmdGate::onHazardStatusStamped, this, _1),
    subscribe_option);

  sub_motion_state_ = this->create_subscription<MotionState>(
    "/api/motion/state",
    rclcpp::QoS{1}.transient_local(),
    std::bind(&EveCmdGate::onMotionState, this, _1),
    subscribe_option);

  // Publisher
  pub_state_ = this->create_publisher<eve_cmd_gate_msgs::msg::EngageRequestState>(
  "/eve_cmd_gate/engage_request_state", rclcpp::QoS{1}.transient_local());

  // Service
  srv_engage_ = this->create_service<tier4_external_api_msgs::srv::Engage>(
    "/api/external/set/engage",
    std::bind(
      &EveCmdGate::execEngageProcess, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, callback_group_service_);

  // Client
  cli_engage_ = this->create_client<tier4_external_api_msgs::srv::Engage>(
    "/api/autoware/set/engage",
    rmw_qos_profile_services_default);
  cli_accept_start_ = this->create_client<autoware_adapi_v1_msgs::srv::AcceptStart>(
    "/api/motion/accept_start",
    rmw_qos_profile_services_default);

  // Variable
  operation_state_.mode = OperationModeState::UNKNOWN;
  operation_state_.is_autoware_control_enabled = false;
  operation_state_.is_in_transition = false;
  operation_state_.is_stop_mode_available = false;
  operation_state_.is_autonomous_mode_available = false;
  operation_state_.is_local_mode_available = false;
  operation_state_.is_remote_mode_available = false;
  current_delivery_reservation_state_ = StateLock::STATE_OFF;
  on_sound_done_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_UNDEFINED;
  on_sound_playing_flg_ = false;
  sound_param_[autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_ENGAGE] =
    {true};
  sound_param_[autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART] =
    {true};
  is_engage_requesting_ = false;
  is_engage_accepted_ = false;
  routing_state_ = RouteState::UNKNOWN;
  routing_route_.data.clear();
  is_emergency_holding_ = false;
  motion_state_ = MotionState::UNKNOWN;
  sound_done_for_restart_ = false;
}

void EveCmdGate::execEngageProcess(
  const tier4_external_api_msgs::srv::Engage::Request::SharedPtr request,
  const tier4_external_api_msgs::srv::Engage::Response::SharedPtr response)
{
  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(),
    *this->get_clock(), DEBUG_THROTTLE_TIME,
    "[eve_cmd_gate] Engage Request Is %d ",
    request->engage);

  if (operation_state_.is_autoware_control_enabled == false) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), DEBUG_THROTTLE_TIME,
      "[eve_cmd_gate] Engage Request Is Not Ready.");
    response->status = tier4_api_utils::response_error("It is not ready to engage.");
    return;
  }

  const auto is_engage_ready_state = isWaitingEngage();

  if (!is_engage_ready_state) {
    // Do not make error notification
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), DEBUG_THROTTLE_TIME,
      "[eve_cmd_gate] Preceding Engage Request");
    response->status = tier4_api_utils::response_error("It is not ready to engage.");
    return;
  }

  uint16_t lock_state;
  {
    std::lock_guard<std::shared_mutex> lock(lock_state_mtx_);
    lock_state = current_delivery_reservation_state_;
  }

  if (lock_state ==
    autoware_state_machine_msgs::msg::StateLock::STATE_VERIFICATION)
  {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(),
      *this->get_clock(), DEBUG_THROTTLE_TIME,
      "[eve_cmd_gate] Under Verification of Lock Button ");
    response->status = tier4_api_utils::response_error("It is not ready to engage.");
    return;
  }

  // Publish engage request state
  setEngageProcess(true, false);

  // Non-blocking async request
  cli_engage_->async_send_request(request,
    [this](rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedFuture future) {
      auto result = future.get();
      if (!tier4_api_utils::is_success(result->status)) {
        RCLCPP_WARN(
          this->get_logger(),
          "[eve_cmd_gate] Engage request failed: %s",
          result->status.message.c_str());
      }
      // Reset engage request state after engage completed
      setEngageProcess(true, false);
    });

  response->status = tier4_api_utils::response_success();
}

void EveCmdGate::setEngageProcess(bool request, bool accept)
{
  is_engage_requesting_ = request;
  is_engage_accepted_ = accept;

  eve_cmd_gate_msgs::msg::EngageRequestState pub;
  pub.is_engage_requesting = is_engage_requesting_;
  pub.is_engage_accepted = is_engage_accepted_;
  pub_state_->publish(pub);
}

std::pair<bool, bool> EveCmdGate::getEngageProcess()
{
  return std::make_pair(is_engage_requesting_, is_engage_accepted_);
}

bool EveCmdGate::isEmergencyHolding(void)
{
  return is_emergency_holding_;
}

bool EveCmdGate::isRequestReset(void)
{
  bool is_request_reset = false;
  auto [is_request, is_accept] = getEngageProcess();
  if (is_request) {
    if ((!isWaitingEngage() && !isDriving()) || isEmergencyHolding()) {
      is_request_reset = true;
    }
  }
  return is_request_reset;
}

void EveCmdGate::onOperationModeStatus(
  const OperationModeState::SharedPtr msg)
{
  operation_state_.mode = msg->mode;
  operation_state_.is_autoware_control_enabled = msg->is_autoware_control_enabled;
  operation_state_.is_in_transition = msg->is_in_transition;
}

void EveCmdGate::onRoutingStatus(
  const RouteState::SharedPtr msg)
{
  routing_state_ = msg->state;
}

void EveCmdGate::onRoutingRoute(
  const Route::SharedPtr msg)
{
  routing_route_.data = msg->data;
}

void EveCmdGate::onLockState(
  const StateLock::SharedPtr msg)
{
  std::lock_guard<std::shared_mutex> lock(lock_state_mtx_);
  current_delivery_reservation_state_ = msg->state;
}

void EveCmdGate::onStateSoundDone(
  const StateSoundDone::SharedPtr msg)
{
  on_sound_done_state_ = msg->state;
  RCLCPP_INFO(this->get_logger(), "[eve_cmd_gate] onStateSoundDone: %d", on_sound_done_state_);

  // Set flag for STATE_INFORM_RESTART (450) or STATE_INFORM_ENGAGE (301)
  if (on_sound_done_state_ ==
    autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART||
    autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_ENGAGE)
  {
    sound_done_for_restart_ = true;
    tryCallAcceptStart();
  }
}

void EveCmdGate::onMotionState(
  const MotionState::SharedPtr msg)
{
  motion_state_ = msg->state;
  RCLCPP_DEBUG(this->get_logger(), "[eve_cmd_gate] onMotionState: %d", motion_state_);
}

void EveCmdGate::tryCallAcceptStart()
{
  // Call /api/motion/accept_start only when:
  // - sound_done for restart has been received
  // - motion state is STARTING
  if (!sound_done_for_restart_) {
    return;
  }

  if (motion_state_ != MotionState::STARTING) {
    RCLCPP_DEBUG(this->get_logger(),
      "[eve_cmd_gate] Waiting for motion state STARTING, current: %d", motion_state_);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "[eve_cmd_gate] Call /api/motion/accept_start");

  if (cli_accept_start_->service_is_ready()) {
    auto request = std::make_shared<autoware_adapi_v1_msgs::srv::AcceptStart::Request>();
    cli_accept_start_->async_send_request(request,
      [this](rclcpp::Client<autoware_adapi_v1_msgs::srv::AcceptStart>::SharedFuture future) {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(),
          "[eve_cmd_gate] /api/motion/accept_start response: %d", response->status.success);
        if (response->status.success) {
          sound_done_for_restart_ = false;
          setEngageProcess(false, true);
        } else {
          RCLCPP_WARN(
            this->get_logger(),
            "[eve_cmd_gate] /api/motion/accept_start failed: %s",
            response->status.message.c_str());
        }
      });
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "[eve_cmd_gate] /api/motion/accept_start service is not ready");
  }
}

void EveCmdGate::onHazardStatusStamped(
  const HazardStatusStamped::SharedPtr msg)
{
  is_emergency_holding_ = msg->status.emergency_holding;
  auto is_engage_requesting_reset = isRequestReset();
  if (is_engage_requesting_reset) {
    setEngageProcess(false, false);
  }
}

bool EveCmdGate::isWaitingEngage()
{
  if ((operation_state_.mode != OperationModeState::AUTONOMOUS)
    && (operation_state_.is_in_transition == false)
    && (operation_state_.is_autoware_control_enabled == true)
    && (routing_route_.data.size() != 0)) {
      return true;
    } else {
      return false;
    }
}

bool EveCmdGate::isDriving()
{
  if ((operation_state_.mode == OperationModeState::AUTONOMOUS)
    && (operation_state_.is_in_transition == false)
    && (operation_state_.is_autoware_control_enabled == true)) {
      return true;
    } else {
      return false;
    }
}

}  // namespace eve_cmd_gate

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(eve_cmd_gate::EveCmdGate)
