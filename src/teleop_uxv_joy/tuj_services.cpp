/**
 * Teleop UXV Joy node services implementation.
 *
 * May 8, 2026
 */

/**
 * Copyright 2026 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <teleop_uxv_joy/teleop_uxv_joy.hpp>

namespace teleop_uxv_joy
{

void TeleopUXVJoy::handle_kill()
{
  RCLCPP_WARN(get_logger(), "Requesting Kill operation");

  // Try to kill-switch the UXV
  Trigger::Request::SharedPtr kill_req = std::make_shared<Trigger::Request>();
  Trigger::Response::SharedPtr kill_res = kill_client_->call_sync(kill_req, false);
  if (kill_res == nullptr) {
    RCLCPP_ERROR(get_logger(), "Kill request not received by server");
  } else {
    if (!(kill_res->success)) {
      RCLCPP_ERROR(get_logger(), "Kill failure: %s", kill_res->message.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Kill success");
    }
  }

  operation_in_progress_.store(false, std::memory_order_release);
}

void TeleopUXVJoy::handle_reset()
{
  RCLCPP_WARN(get_logger(), "Requesting Reset operation");

  // Try to reset the UXV
  Trigger::Request::SharedPtr reset_req = std::make_shared<Trigger::Request>();
  Trigger::Response::SharedPtr reset_res = reset_client_->call_sync(reset_req, false);
  if (reset_res == nullptr) {
    RCLCPP_ERROR(get_logger(), "Reset request not received by server");
  } else {
    if (!(reset_res->success)) {
      RCLCPP_ERROR(get_logger(), "Reset failure: %s", reset_res->message.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Reset success");
    }
  }

  operation_in_progress_.store(false, std::memory_order_release);
}

} // namespace teleop_uxv_joy
