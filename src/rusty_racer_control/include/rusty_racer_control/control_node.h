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
#include "rusty_racer_interfaces/msg/traffic_sign.hpp"

// Project component headers
#include "rusty_racer_control/laengsfuehrung_controller.h"
#include "rusty_racer_control/lateral_controller.h"
#include "rusty_racer_control/traffic_fsm2.h"

/**
 * @class ControlNode
 * @brief Main control node for longitudinal and lateral control
 *
 * Two velocity modes (selected at runtime via cruise_mode parameter):
 *   - cruise_mode=true:  Gate-Only cruise (wait for start gate, then constant v_ref)
 *   - cruise_mode=false: Full FSM (three-layer traffic sign logic)
 *
 * No curvature feedforward; lateral controller uses 3-param PD.
 */
class ControlNode: public rclcpp::Node {
public:
  ControlNode();

private:
  // -- Callbacks -------------------------------------------------------
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void laneCallback(
    const rusty_racer_interfaces::msg::LaneDeviation::SharedPtr msg);

  void trafficSignCallback(
    const rusty_racer_interfaces::msg::TrafficSign::SharedPtr msg);

  // -- Controllers -----------------------------------------------------
  std::unique_ptr < LateralController > lateral_controller_;
  PIParams pi_params_;
  PIState pi_state_;

  // -- Traffic decision system -----------------------------------------
  TrafficFSM traffic_fsm_;
  TrafficParams traffic_params_;
  CamFrame latest_cam_frame_;
  rclcpp::Time last_traffic_update_;

  // Timeout for FSM sign input [seconds].
  static constexpr double kTrafficSignTimeout = 2.5;

  // -- Mode selection --------------------------------------------------
  bool cruise_mode_;           // true = Gate-Only cruise, false = Full FSM

  // Cruise-mode gate state (two-phase)
  bool cruise_gate_unlocked_;    // false = waiting at start gate, true = cruising
  bool cruise_stop_seen_;        // Phase 1: Stop sign confirmed present
  int cruise_stop_on_count_;     // Phase 1: consecutive frames with Stop
  int cruise_stop_gone_count_;   // Phase 2: consecutive frames without Stop

  // -- Target values ---------------------------------------------------
  double v_ref_;       // Cruise speed [m/s]:
                       //   cruise_mode_=true  -> fixed speed after gate
                       //   cruise_mode_=false -> unused (FSM decides)
  double y_target_;    // Lateral offset target [m]

  // -- Current state ---------------------------------------------------
  double current_v_;
  double current_psi_k_;
  rclcpp::Time last_update_time_;
  double motor_level_smooth_ = 0.0;  // Smoothed motor command [0,1]

  // -- ROS2 communication ----------------------------------------------
  rclcpp::Subscription < nav_msgs::msg::Odometry > ::SharedPtr odom_sub_;
  rclcpp::Subscription < rusty_racer_interfaces::msg::LaneDeviation > ::SharedPtr lane_sub_;
  rclcpp::Subscription < rusty_racer_interfaces::msg::TrafficSign > ::SharedPtr traffic_sign_sub_;
  rclcpp::Publisher < rusty_racer_interfaces::msg::MotorCommand > ::SharedPtr motor_cmd_pub_;
};

#endif  // RUSTY_RACER_CONTROL__CONTROL_NODE_H_
