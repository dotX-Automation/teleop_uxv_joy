/**
 * Teleop UXV Joy node implementation.
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

TeleopUXVJoy::TeleopUXVJoy(const rclcpp::NodeOptions & node_options)
: NodeBase("teleop_uxv_joy", node_options, true)
{
  dua_init_node();

  RCLCPP_INFO(get_logger(), "Node initialized");
}

TeleopUXVJoy::~TeleopUXVJoy()
{
  // Wait for pending operations to complete
  if (op_thread_.joinable()) {
    op_thread_.join();
  }
}

void TeleopUXVJoy::init_cgroups()
{
  joy_sub_cgroup_ = dua_create_exclusive_cgroup();
}

void TeleopUXVJoy::init_subscribers()
{
  // joy
  joy_sub_ = dua_create_subscription<Joy>(
    "/joy",
    std::bind(
      &TeleopUXVJoy::joy_sub_clbk,
      this,
      std::placeholders::_1),
    dua_qos::Reliable::get_datum_qos(),
    joy_sub_cgroup_);
}

void TeleopUXVJoy::init_publishers()
{
  // cmd_op
  if (actions_arm_as_topic_   || actions_disarm_as_topic_ ||
      services_kill_as_topic_ || services_reset_as_topic_) {
    cmd_op_pub_ = dua_create_publisher<String>(
      "/cmd_op",
      dua_qos::Reliable::get_datum_qos());
  }

  // cmd_uxv
  cmd_uxv_pub_ = dua_create_publisher<UXVCommand>(
    "/cmd_uxv",
    dua_qos::Reliable::get_datum_qos());
}

void TeleopUXVJoy::init_service_clients()
{
  // kill
  if (!services_kill_as_topic_ && !services_kill_name_.empty()) {
    kill_client_ = dua_create_service_client<Trigger>(services_kill_name_, wait_servers_);
  }

  // reset
  if (!services_reset_as_topic_ && !services_reset_name_.empty()) {
    reset_client_ = dua_create_service_client<Trigger>(services_reset_name_, wait_servers_);
  }
}

void TeleopUXVJoy::init_action_clients()
{
  // arm
  if (!actions_arm_as_topic_ && !actions_arm_name_.empty()) {
    arm_client_ = dua_create_action_client<Arm>(actions_arm_name_, nullptr, wait_servers_);
  }

  // disarm
  if (!actions_disarm_as_topic_ && !actions_disarm_name_.empty()) {
    disarm_client_ = dua_create_action_client<Disarm>(actions_disarm_name_, nullptr, wait_servers_);
  }
}

} // namespace teleop_uxv_joy

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(teleop_uxv_joy::TeleopUXVJoy)
