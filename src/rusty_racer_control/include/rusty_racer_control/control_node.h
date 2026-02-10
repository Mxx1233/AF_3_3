// Copyright 2025 Rusty Racer Team
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

/**
 * @file control_node.h
 * @brief Rusty Racer control node header
 * @author zx
 * @date 2025-12
 */

#ifndef RUSTY_RACER_CONTROL__CONTROL_NODE_H_
#define RUSTY_RACER_CONTROL__CONTROL_NODE_H_

// C++ system headers
#include <memory>

// ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

// Project interface headers
#include "rusty_racer_interfaces/msg/lane_deviation.hpp"
#include "rusty_racer_interfaces/msg/motor_command.hpp"

// Project component headers
#include "rusty_racer_control/laengsfuehrung_controller.h"
#include "rusty_racer_control/lateral_controller.h"

/**
 * @class ControlNode
 * @brief Main control node for longitudinal and lateral control
 */
class ControlNode: public rclcpp::Node {
public:
  /**
   * @brief Constructor
   */
  ControlNode();

private:
  /**
   * @brief Odometry callback
   * @param msg Odometry message
   */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Lane deviation callback - main control loop
   * @param msg Lane deviation message
   */
  void laneCallback(
    const rusty_racer_interfaces::msg::LaneDeviation::SharedPtr msg);

  // Controllers
  std::unique_ptr < LateralController > lateral_controller_;
  PIParams pi_params_;
  PIState pi_state_;

  // Target values
  double v_ref_;      ///< Target velocity [m/s]
  double y_target_;   ///< Target lateral offset [m]

  // Current state
  double current_v_;       ///< Current velocity [m/s]
  double current_psi_k_;   ///< Current yaw angle [rad]
  rclcpp::Time last_update_time_;  ///< Last update timestamp

  // ROS2 communication
  rclcpp::Subscription < nav_msgs::msg::Odometry > ::SharedPtr odom_sub_;
  rclcpp::Subscription < rusty_racer_interfaces::msg::LaneDeviation > ::SharedPtr
  lane_sub_;
  rclcpp::Publisher < rusty_racer_interfaces::msg::MotorCommand > ::SharedPtr
  motor_cmd_pub_;
};

#endif  // RUSTY_RACER_CONTROL__CONTROL_NODE_H_
