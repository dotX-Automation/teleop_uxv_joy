/**
 * Teleop UXV Joy node actions implementation.
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

void TeleopUXVJoy::handle_arm()
{
  armed_.store(true, std::memory_order_release); // No matter what the server will say, this also changes the internal state of this module
  RCLCPP_WARN(get_logger(), "ARM");

  // Try to arm the UXV
  Arm::Goal arm_goal{};
  auto arm_res = arm_client_->call_sync(arm_goal, false);
  bool arm_accepted = std::get<0>(arm_res);
  if (!arm_accepted) {
    RCLCPP_ERROR(get_logger(), "Arm request not accepted by server");
  } else {
    rclcpp_action::ResultCode res_code = std::get<1>(arm_res);
    auto arm_result_ptr = std::get<2>(arm_res);
    if ((res_code == rclcpp_action::ResultCode::SUCCEEDED ||
         res_code == rclcpp_action::ResultCode::UNKNOWN)  &&
        arm_result_ptr->result.result == CommandResultStamped::SUCCESS)
    {
      RCLCPP_INFO(get_logger(), "Arm success");
    } else {
      RCLCPP_ERROR(get_logger(), "Arm failure");
    }
  }

  operation_in_progress_.store(false, std::memory_order_release);
}

void TeleopUXVJoy::handle_disarm()
{
  armed_.store(false, std::memory_order_release); // No matter what the server will say, this also changes the internal state of this module
  RCLCPP_WARN(get_logger(), "DISARM");

  // Try to disarm the UXV
  Disarm::Goal disarm_goal{};
  auto disarm_res = disarm_client_->call_sync(disarm_goal, false);
  bool disarm_accepted = std::get<0>(disarm_res);
  if (!disarm_accepted) {
    RCLCPP_ERROR(get_logger(), "Disarm request not accepted by server");
  } else {
    rclcpp_action::ResultCode res_code = std::get<1>(disarm_res);
    auto disarm_result_ptr = std::get<2>(disarm_res);
    if ((res_code == rclcpp_action::ResultCode::SUCCEEDED ||
         res_code == rclcpp_action::ResultCode::UNKNOWN)  &&
        disarm_result_ptr->result.result == CommandResultStamped::SUCCESS)
    {
      RCLCPP_INFO(get_logger(), "Disarm success");
    } else {
      RCLCPP_ERROR(get_logger(), "Disarm failure");
    }
  }

  operation_in_progress_.store(false, std::memory_order_release);
}

} // namespace teleop_uxv_joy
