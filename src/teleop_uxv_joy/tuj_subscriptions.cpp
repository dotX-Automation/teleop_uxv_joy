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
  if (operation_in_progress_.load(std::memory_order_acquire)) {
    return;
  }

  // Clean up previous executions
  if (op_thread_.joinable()) {
    op_thread_.join();
  }

  rclcpp::Time now_ts = commands_clock_.now();

  // Get gear commands (with cooldown)
  if (now_ts - gear_last_ts_ > rclcpp::Duration(std::chrono::nanoseconds(gear_cooldown_ * 1000000))) {
    if (gear_automatic_) {
      // Get gear N
      if (gear_n_index_ != INDEX_INVALID && static_cast<std::size_t>(gear_n_index_) < msg->buttons.size() &&
          msg->buttons[gear_n_index_] == BUTTON_PRESSED) {
        gear_ = UXVGear::GEAR_N;
        gear_last_ts_ = commands_clock_.now();
        RCLCPP_WARN(get_logger(), "GEAR %s", get_gear_str(gear_).c_str());
        goto gears_done;
      }

      // Get gear D
      if (gear_d_index_ != INDEX_INVALID && static_cast<std::size_t>(gear_d_index_) < msg->buttons.size() &&
          msg->buttons[gear_d_index_] == BUTTON_PRESSED) {
        gear_ = UXVGear::GEAR_D;
        gear_last_ts_ = commands_clock_.now();
        RCLCPP_WARN(get_logger(), "GEAR %s", get_gear_str(gear_).c_str());
        goto gears_done;
      }

      // Get gear R
      if (gear_r_index_ != INDEX_INVALID && static_cast<std::size_t>(gear_r_index_) < msg->buttons.size() &&
          msg->buttons[gear_r_index_] == BUTTON_PRESSED) {
        gear_ = UXVGear::GEAR_R;
        gear_last_ts_ = commands_clock_.now();
        RCLCPP_WARN(get_logger(), "GEAR %s", get_gear_str(gear_).c_str());
        goto gears_done;
      }
    } else {
      // Get gear down
      if (gear_down_index_ != INDEX_INVALID && static_cast<std::size_t>(gear_down_index_) < msg->buttons.size() &&
          msg->buttons[gear_down_index_] == BUTTON_PRESSED) {
        gear_ -= 1;
        gear_last_ts_ = commands_clock_.now();
        RCLCPP_WARN(get_logger(), "GEAR %s", get_gear_str(gear_).c_str());
        goto gears_done;
      }

      // Get gear up
      if (gear_up_index_ != INDEX_INVALID && static_cast<std::size_t>(gear_up_index_) < msg->buttons.size() &&
          msg->buttons[gear_up_index_] == BUTTON_PRESSED) {
        gear_ += 1;
        gear_last_ts_ = commands_clock_.now();
        RCLCPP_WARN(get_logger(), "GEAR %s", get_gear_str(gear_).c_str());
        goto gears_done;
      }
    }
  }
gears_done:

  // Get service commands (with cooldown)
  if (now_ts - services_last_ts_ > rclcpp::Duration(std::chrono::nanoseconds(services_cooldown_ * 1000000))) {
    // Check for kill service
    if (static_cast<std::size_t>(services_kill_index_) < msg->buttons.size() &&
        msg->buttons[services_kill_index_] == BUTTON_PRESSED) {
      if (services_kill_as_topic_) {
        String cmd_op_msg{};
        cmd_op_msg.set__data("kill");
        cmd_op_pub_->publish(cmd_op_msg);
      } else if (!services_kill_name_.empty()) {
        operation_in_progress_.store(true, std::memory_order_release);
        op_thread_ = std::thread(
          std::bind(
            &TeleopUXVJoy::handle_kill,
            this));
        services_last_ts_ = commands_clock_.now();
      }
      return;
    }

    // Check for reset service
    if (static_cast<std::size_t>(services_reset_index_) < msg->buttons.size() &&
        msg->buttons[services_reset_index_] == BUTTON_PRESSED) {
      if (services_reset_as_topic_) {
        String cmd_op_msg{};
        cmd_op_msg.set__data("reset");
        cmd_op_pub_->publish(cmd_op_msg);
      } else if (!services_reset_name_.empty()) {
        operation_in_progress_.store(true, std::memory_order_release);
        op_thread_ = std::thread(
          std::bind(
            &TeleopUXVJoy::handle_reset,
            this));
        services_last_ts_ = commands_clock_.now();
      }
      return;
    }
  }

  // Get action  commands (with cooldown)
  if (now_ts - actions_last_ts_ > rclcpp::Duration(std::chrono::nanoseconds(actions_cooldown_ * 1000000))) {
    // Check for arm action
    if (static_cast<std::size_t>(actions_arm_index_) < msg->buttons.size() &&
        msg->buttons[actions_arm_index_] == BUTTON_PRESSED) {
      if (actions_arm_as_topic_) {
        String cmd_op_msg{};
        cmd_op_msg.set__data("arm");
        cmd_op_pub_->publish(cmd_op_msg);
      } else if (!actions_arm_name_.empty()) {
        operation_in_progress_.store(true, std::memory_order_release);
        op_thread_ = std::thread(
          std::bind(
            &TeleopUXVJoy::handle_arm,
            this));
        actions_last_ts_ = commands_clock_.now();
      }
      return;
    }

    // Check for disarm action
    if (static_cast<std::size_t>(actions_disarm_index_) < msg->buttons.size() &&
        msg->buttons[actions_disarm_index_] == BUTTON_PRESSED) {
      if (actions_disarm_as_topic_) {
        String cmd_op_msg{};
        cmd_op_msg.set__data("disarm");
        cmd_op_pub_->publish(cmd_op_msg);
      } else if (!actions_disarm_name_.empty()) {
        operation_in_progress_.store(true, std::memory_order_release);
        op_thread_ = std::thread(
          std::bind(
            &TeleopUXVJoy::handle_disarm,
            this));
        actions_last_ts_ = commands_clock_.now();
      }
      return;
    }
  }

  // Check enable button
  if (enable_button_require_) {
    if (static_cast<std::size_t>(enable_button_index_) >= msg->buttons.size() ||
        msg->buttons[enable_button_index_] == BUTTON_RELEASED) {
      return;
    }
  }

  // Get axes
  float lh = axes_lh_index_ != INDEX_INVALID && static_cast<std::size_t>(axes_lh_index_) < msg->axes.size() ?
             msg->axes[axes_lh_index_] : AXIS_NEUTRAL;
  float lv = axes_lv_index_ != INDEX_INVALID && static_cast<std::size_t>(axes_lv_index_) < msg->axes.size() ?
             msg->axes[axes_lv_index_] : AXIS_NEUTRAL;
  float rh = axes_rh_index_ != INDEX_INVALID && static_cast<std::size_t>(axes_rh_index_) < msg->axes.size() ?
             msg->axes[axes_rh_index_] : AXIS_NEUTRAL;
  float rv = axes_rv_index_ != INDEX_INVALID && static_cast<std::size_t>(axes_rv_index_) < msg->axes.size() ?
             msg->axes[axes_rv_index_] : AXIS_NEUTRAL;
  if (std::fabs(lh) < static_cast<float>(axes_lh_deadzone_)) lh = AXIS_NEUTRAL;
  if (std::fabs(lv) < static_cast<float>(axes_lv_deadzone_)) lv = AXIS_NEUTRAL;
  if (std::fabs(rh) < static_cast<float>(axes_rh_deadzone_)) rh = AXIS_NEUTRAL;
  if (std::fabs(rv) < static_cast<float>(axes_rv_deadzone_)) rv = AXIS_NEUTRAL;
  lh *= static_cast<float>(axes_lh_scale_);
  lv *= static_cast<float>(axes_lv_scale_);
  rh *= static_cast<float>(axes_rh_scale_);
  rv *= static_cast<float>(axes_rv_scale_);
  if (axes_lh_reverse_) lh *= AXIS_REVERSE;
  if (axes_lv_reverse_) lv *= AXIS_REVERSE;
  if (axes_rh_reverse_) rh *= AXIS_REVERSE;
  if (axes_rv_reverse_) rv *= AXIS_REVERSE;
  const float lh0 = lh, lv0 = lv, rh0 = rh, rv0 = rv;
  if (axes_lh_normalize_round_) lh = sgn(lh0) * std::min(std::hypot(lh0, lv0), 1.0f);
  if (axes_lv_normalize_round_) lv = sgn(lv0) * std::min(std::hypot(lh0, lv0), 1.0f);
  if (axes_rh_normalize_round_) rh = sgn(rh0) * std::min(std::hypot(rh0, rv0), 1.0f);
  if (axes_rv_normalize_round_) rv = sgn(rv0) * std::min(std::hypot(rh0, rv0), 1.0f);

  // Smooth the axes with a 1€ (One-Euro) adaptive low-pass filter to remove twitching
  if (axes_filter_enable_) {
    const float dt = static_cast<float>((now_ts - axes_filter_last_ts_).seconds());
    lh = filter_axis(lh, dt, axes_filter_state_[0]);
    lv = filter_axis(lv, dt, axes_filter_state_[1]);
    rh = filter_axis(rh, dt, axes_filter_state_[2]);
    rv = filter_axis(rv, dt, axes_filter_state_[3]);
    axes_filter_last_ts_ = now_ts;
  }

  // Get numerical inputs
  std::array<bool, UXVNumChannels::N_CHANNELS> num_inputs_valid;
  std::array<int16_t, UXVNumChannels::N_CHANNELS> num_inputs;
  num_inputs_valid.fill(false);
  num_inputs.fill(UXVNumChannels::CHAN_OFF);
  for (std::size_t i = 0; i < std::min(num_inputs_indexes_.size(), static_cast<std::size_t>(UXVNumChannels::N_CHANNELS)); i++) {
    if (static_cast<std::size_t>(num_inputs_indexes_[i]) >= msg->buttons.size()) {
      continue;
    }

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
  cmd_uxv_msg.op_mode.set__mode(static_cast<int16_t>(uxv_command_mode_));
  cmd_uxv_msg.set__arm(uxv_command_arm_);
  cmd_uxv_msg.set__reset(false);
  cmd_uxv_pub_->publish(cmd_uxv_msg);
}

} // namespace teleop_uxv_joy
