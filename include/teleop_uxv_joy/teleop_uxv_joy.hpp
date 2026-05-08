/**
 * Teleop UXV Joy node definition.
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

#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <dua_node_cpp/dua_node.hpp>
#include <dua_qos_cpp/dua_qos.hpp>

#include <dua_common_interfaces/msg/command_result_stamped.hpp>
#include <dua_uxv_interfaces/msg/uxv_command.hpp>
#include <dua_uxv_interfaces/msg/uxv_gear.hpp>
#include <dua_uxv_interfaces/msg/uxv_mode.hpp>
#include <dua_uxv_interfaces/msg/uxv_num_channels.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/header.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <dua_hardware_interfaces/action/arm.hpp>
#include <dua_hardware_interfaces/action/disarm.hpp>

namespace teleop_uxv_joy
{

using CommandResultStamped = dua_common_interfaces::msg::CommandResultStamped;
using UXVCommand           = dua_uxv_interfaces::msg::UXVCommand;
using UXVGear              = dua_uxv_interfaces::msg::UXVGear;
using UXVMode              = dua_uxv_interfaces::msg::UXVMode;
using UXVNumChannels       = dua_uxv_interfaces::msg::UXVNumChannels;
using Joy                  = sensor_msgs::msg::Joy;
using Header               = std_msgs::msg::Header;

using Trigger              = std_srvs::srv::Trigger;

using Arm                  = dua_hardware_interfaces::action::Arm;
using Disarm               = dua_hardware_interfaces::action::Disarm;

/**
 * Transforms joystick samples into UXV commands of various nature.
 */
class TeleopUXVJoy : public dua_node::NodeBase
{
public:
  /**
   * @brief Constructor.
   *
   * @param node_options Node options.
   */
  explicit TeleopUXVJoy(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  /**
   * @brief Destructor.
   */
  virtual ~TeleopUXVJoy();

private:
  /* Node initialization routines. */
  /**
   * @brief Initializes node parameters.
   */
  void init_parameters() override;

  /**
   * @brief Initializes callback groups.
   */
  void init_cgroups() override;

  /**
   * @brief Initializes subscriptions.
   */
  void init_subscribers() override;

  /**
   * @brief Initializes publishers.
   */
  void init_publishers() override;

  /**
   * @brief Initializes service clients.
   */
  void init_service_clients() override;

  /**
   * @brief Initializes action clients.
   */
  void init_action_clients() override;

  /* Topic subscription callback groups. */
  rclcpp::CallbackGroup::SharedPtr joy_sub_cgroup_;

  /* Topic subscriptions. */
  rclcpp::Subscription<Joy>::SharedPtr joy_sub_;

  /* Topic subscription callbacks. */
  /**
   * @brief Processes incoming Joy messages and takes appropriate actions.
   *
   * @param msg Incoming Joy message to parse.
   */
  void joy_sub_clbk(const Joy::ConstSharedPtr msg);

  /* Topic publishers. */
  rclcpp::Publisher<UXVCommand>::SharedPtr cmd_uxv_pub_;

  /* Service clients. */
  simple_serviceclient::Client<Trigger>::SharedPtr kill_client_;
  simple_serviceclient::Client<Trigger>::SharedPtr reset_client_;

  /* Action clients. */
  simple_actionclient::Client<Arm>::SharedPtr arm_client_;
  simple_actionclient::Client<Disarm>::SharedPtr disarm_client_;

  /* Thread routines. */
  /**
   * @brief Handles an Arm operation.
   */
  void handle_arm();

  /**
   * @brief Handes a Disarm operation.
   */
  void handle_disarm();

  /**
   * @brief Handles a Kill operation.
   */
  void handle_kill();

  /**
   * @brief Handles a Reset operation.
   */
  void handle_reset();

  /* Synchronization primitives. */
  std::mutex operation_lock_;

  /* Internal state variables. */
  bool    armed_ = false;
  int16_t gear_ = UXVGear::GEAR_N;

  /* Node parameters. */
  int64_t              actions_arm_index_;
  std::string          actions_arm_name_;
  int64_t              actions_disarm_index_;
  std::string          actions_disarm_name_;
  int64_t              axes_lh_index_;
  double               axes_lh_scale_;
  int64_t              axes_lv_index_;
  double               axes_lv_scale_;
  int64_t              axes_rh_index_;
  double               axes_rh_scale_;
  int64_t              axes_rv_index_;
  double               axes_rv_scale_;
  int64_t              enable_button_index_;
  bool                 enable_button_require_;
  std::string          frame_id_;
  int64_t              gear_down_index_;
  int64_t              gear_up_index_;
  std::vector<int64_t> num_inputs_indexes_;
  int64_t              services_kill_index_;
  std::string          services_kill_name_;
  int64_t              services_reset_index_;
  std::string          services_reset_name_;
  bool                 wait_servers_;

  static constexpr int BUTTON_RELEASED = 0;
  static constexpr int BUTTON_PRESSED  = 1;
  static constexpr int INDEX_INVALID   = -1;
};

} // namespace teleop_uxv_joy
