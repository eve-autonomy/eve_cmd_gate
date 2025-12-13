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
  // This type of callback group only allows one callback to be executed at a time
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
    subscribe_option
  );

  sub_routing_state_ = this->create_subscription<RouteState>(
    "/api/routing/state",
    rclcpp::QoS{1}.transient_local(),
    std::bind(&EveCmdGate::onRoutingStatus, this, _1),
    subscribe_option
  );

  sub_routing_route_ = this->create_subscription<Route>(
    "/api/routing/route",
    rclcpp::QoS{1}.transient_local(),
    std::bind(&EveCmdGate::onRoutingRoute, this, _1),
    subscribe_option
  );

  sub_lock_state_ = this->create_subscription<StateLock>(
    "/go_interface/lock_state",
    rclcpp::QoS{1}.transient_local(),
    std::bind(&EveCmdGate::onLockState, this, _1),
    subscribe_option
  );

  sub_engage_sound_done_ = this->create_subscription<StateSoundDone>(
    "/autoware_state_machine/state_sound_done", rclcpp::QoS{1}.transient_local(),
    std::bind(&EveCmdGate::onStateSoundDone, this, std::placeholders::_1),
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
  srv_set_request_start_api_ = this->create_service<std_srvs::srv::Trigger>(
    "/api/autoware/set/start_request",
    std::bind(
      &EveCmdGate::setRequestStartAPI, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, callback_group_service_);

  // Client
  cli_engage_ = this->create_client<tier4_external_api_msgs::srv::Engage>(
    "/api/autoware/set/engage",
    rmw_qos_profile_services_default);
  cli_set_operator_ = this->create_client<tier4_external_api_msgs::srv::SetOperator>(
    "/api/autoware/set/operator",
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

  // Provisional support
  // Set operator to AUTONOMOUS and set "/vehicle/engage" to True internally.
  auto operator_req =
    std::make_shared<tier4_external_api_msgs::srv::SetOperator::Request>();
  operator_req->mode.mode = tier4_external_api_msgs::msg::Operator::AUTONOMOUS;
  auto operator_future = cli_set_operator_->async_send_request(operator_req);
  if (!tier4_api_utils::is_success(operator_future.get()->status)) {
    response->status = tier4_api_utils::response_error("Operator set failed.");
    return;
  }

  bool is_success = waitingForEngageAccept();
  if (!is_success) {
    response->status = tier4_api_utils::response_error("It is not ready to engage.");
    return;
  }

  auto engage_future = cli_engage_->async_send_request(request);
  response->status = engage_future.get()->status;
}

void EveCmdGate::setRequestStartAPI(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  const std_srvs::srv::Trigger::Response::SharedPtr response
)
{
  /* The voice guidance at the time of engage is done in execEngageProcess function,
      so there is no need to do it there.
    Treats the state as ready to engage and returns true. */
  if (isWaitingEngage() &&
    (on_sound_done_state_ ==
    autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_ENGAGE) &&
    (on_sound_playing_flg_ == false))
  {
    response->success = true;
    return;
  }

  if (operation_state_.is_autoware_control_enabled == false) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), DEBUG_THROTTLE_TIME,
      "[eve_cmd_gate] Start API Is Not Ready ");
    response->success = false;
    return;
  }

  const auto is_running_state = isDriving();

  if (!is_running_state) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), DEBUG_THROTTLE_TIME,
      "[eve_cmd_gate] Start API Is Not Ready ");
    response->success = false;
    return;
  }

  const auto is_exception_state = (on_sound_playing_flg_ == true);

  // Remove exceptions that fall within the scope of RunningState.
  if (is_exception_state) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), DEBUG_THROTTLE_TIME,
      "[eve_cmd_gate] Start API Is Not Ready ");
    response->success = false;
    return;
  }

  bool is_success = waitingForEngageAccept();
  response->success = is_success;
}

bool EveCmdGate::waitingForEngageAccept()
{
  setEngageProcess(true, false);
  on_sound_playing_flg_ = true;

  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
    auto [is_request, is_accept] = getEngageProcess();
    if (is_accept) {
      break;
    }
    if (!is_request) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(), DEBUG_THROTTLE_TIME,
        "[eve_cmd_gate] Engage Request Interruption ");
      return false;
    }
    rate.sleep();
  }
  setEngageProcess(false, false);
  return true;
}

void EveCmdGate::setEngageProcess(bool request, bool accept)
{
  /* In multi-threaded system, there is a possibility of simultaneous accesses from different threads,
      so exclusion control is performed.
     Set request to "true" when we want the audio to play when vehicle departs and restarts.
     Set accept to "true" when the audio playback is complete.
     If you want to suspend waiting for playback to complete for some reason,
      set both values to "false". */
  {
    std::lock_guard<std::shared_mutex> lock(engage_mtx_);
    is_engage_requesting_ = request;
    is_engage_accepted_ = accept;
  }
  eve_cmd_gate_msgs::msg::EngageRequestState pub;
  pub.is_engage_requesting = is_engage_requesting_;
  pub.is_engage_accepted = is_engage_accepted_;
  pub_state_->publish(pub);
}

std::pair<bool, bool> EveCmdGate::getEngageProcess()
{
  /* In multi-threaded system, there is a possibility of simultaneous accesses from different threads,
      so exclusion control is performed.
     Check both values to determine if the playback is played, interrupted, or completed. */
  std::pair<bool, bool> value;
  {
    std::shared_lock<std::shared_mutex> lock(engage_mtx_);
    value = std::make_pair(is_engage_requesting_, is_engage_accepted_);
  }
  return value;
}

bool EveCmdGate::isRequestReset(void)
{
  bool is_request_reset = false;
  auto [is_request, is_accept] = getEngageProcess();
  if (is_request) {
    if ((!isWaitingEngage() && !isDriving())) {
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
  auto is_engage_requesting_reset = isRequestReset();
  if (is_engage_requesting_reset) {
    setEngageProcess(false, false);
  }
}

void EveCmdGate::onRoutingStatus(
  const RouteState::SharedPtr msg)
{
  routing_state_ = msg->state;
  auto is_engage_requesting_reset = isRequestReset();
  if (is_engage_requesting_reset) {
    setEngageProcess(false, false);
  }
}

void EveCmdGate::onRoutingRoute(
  const Route::SharedPtr msg)
{
  routing_route_.data = msg->data;
  auto is_engage_requesting_reset = isRequestReset();
  if (is_engage_requesting_reset) {
    setEngageProcess(false, false);
  }
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
  if (sound_param_.count(on_sound_done_state_) != 0) {
    on_sound_playing_flg_ = false;
    setEngageProcess(false, true);
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
