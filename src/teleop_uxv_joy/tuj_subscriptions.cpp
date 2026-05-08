/**
 * Teleop UXV Joy node subscriptions implementation.
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

void TeleopUXVJoy::joy_sub_clbk(const Joy::ConstSharedPtr msg)
{
  // Check if an operation is in progress
  std::unique_lock<std::mutex> op_lock(operation_lock_, std::try_to_lock);
  if (!op_lock.owns_lock()) {
    return;
  }

  // Check enable button
  if (enable_button_require_) {
    if (msg->buttons[enable_button_index_] == BUTTON_RELEASED) {
      return;
    }
  }

  // Check for kill service
  if (!services_kill_name_.empty()) {
    if (msg->buttons[services_kill_index_] == BUTTON_PRESSED) {
      std::thread(
        std::bind(
          &TeleopUXVJoy::handle_kill,
          this)).detach();
      return;
    }
  }

  // Check for reset service
  if (!services_reset_name_.empty()) {
    if (msg->buttons[services_reset_index_] == BUTTON_PRESSED) {
      std::thread(
        std::bind(
          &TeleopUXVJoy::handle_reset,
          this)).detach();
      return;
    }
  }

  // Check for arm action
  if (!actions_arm_name_.empty()) {
    if (msg->buttons[actions_arm_index_] == BUTTON_PRESSED) {
      std::thread(
        std::bind(
          &TeleopUXVJoy::handle_arm,
          this)).detach();
      return;
    }
  }

  // Check for disarm action
  if (!actions_disarm_name_.empty()) {
    if (msg->buttons[actions_disarm_index_] == BUTTON_PRESSED) {
      std::thread(
        std::bind(
          &TeleopUXVJoy::handle_disarm,
          this)).detach();
      return;
    }
  }

  // Get axes
  float lh = axes_lh_index_ != INDEX_INVALID ? msg->axes[axes_lh_index_] : 0.0f;
  float lv = axes_lv_index_ != INDEX_INVALID ? msg->axes[axes_lv_index_] : 0.0f;
  float rh = axes_rh_index_ != INDEX_INVALID ? msg->axes[axes_rh_index_] : 0.0f;
  float rv = axes_rv_index_ != INDEX_INVALID ? msg->axes[axes_rv_index_] : 0.0f;
  lh *= static_cast<float>(axes_lh_scale_);
  lv *= static_cast<float>(axes_lv_scale_);
  rh *= static_cast<float>(axes_rh_scale_);
  rv *= static_cast<float>(axes_rv_scale_);

  // Get gear down
  if (gear_down_index_ != INDEX_INVALID && msg->buttons[gear_down_index_] == BUTTON_PRESSED) {
    gear_ = std::clamp(static_cast<int16_t>(gear_ - 1), UXVGear::GEAR_R, UXVGear::GEAR_D);
    RCLCPP_WARN(get_logger(), "GEAR %hd", gear_);
  }

  // Get gear up
  if (gear_up_index_ != INDEX_INVALID && msg->buttons[gear_up_index_] == BUTTON_PRESSED) {
    gear_ = std::clamp(static_cast<int16_t>(gear_ + 1), UXVGear::GEAR_R, UXVGear::GEAR_D);
    RCLCPP_WARN(get_logger(), "GEAR %hd", gear_);
  }

  // Get numerical inputs
  std::array<bool, UXVNumChannels::N_CHANNELS> num_inputs_valid;
  std::array<int16_t, UXVNumChannels::N_CHANNELS> num_inputs;
  num_inputs_valid.fill(false);
  num_inputs.fill(UXVNumChannels::CHAN_OFF);
  for (std::size_t i = 0; i < num_inputs_indexes_.size(); i++) {
    num_inputs_valid[i] = true;
    num_inputs[i] = msg->buttons[num_inputs_indexes_[i]] == BUTTON_PRESSED ?
                    UXVNumChannels::CHAN_ON : UXVNumChannels::CHAN_OFF;
  }

  // Fill and publish UXVCommand message
  UXVCommand cmd_uxv_msg{};
  cmd_uxv_msg.header.set__frame_id(frame_id_);
  cmd_uxv_msg.header.set__stamp(get_clock()->now());
  cmd_uxv_msg.set__lstick_v(lv);
  cmd_uxv_msg.set__lstick_h(lh);
  cmd_uxv_msg.set__rstick_v(rv);
  cmd_uxv_msg.set__rstick_h(rh);
  cmd_uxv_msg.act_gear.set__gear(gear_);
  cmd_uxv_msg.num_inputs.set__channels(num_inputs);
  cmd_uxv_msg.num_inputs.set__valid(num_inputs_valid);
  cmd_uxv_msg.op_mode.set__mode(UXVMode::UXV_MODE_OFFBOARD);
  cmd_uxv_msg.set__arm(armed_);
  cmd_uxv_msg.set__reset(false);
  cmd_uxv_pub_->publish(cmd_uxv_msg);
}

} // namespace teleop_uxv_joy
