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
#include <sensor_msgs/msg/image.hpp>

// OpenCV / cv_bridge headers
#include <cv_bridge/cv_bridge.h>  // NOLINT(build/include_order)
#include <opencv2/opencv.hpp>

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
 * Dual-mode velocity source (selected at runtime via traffic_enabled_ param):
 *   - false: pure lane-following, v_ref = fixed cruise speed
 *   - true:  FSM mode, v_ref = TrafficFSM decision output
 *
 * No curvature feedforward; lateral controller uses 3-param PD.
 */
class ControlNode: public rclcpp::Node {
public:
  ControlNode();

private:
  // -- Callbacks -------------------------------------------------------
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

<<<<<<< HEAD
  /**
   * @brief Main control loop (triggered by Lane Deviation)
   */
=======
>>>>>>> eabb7b2 (feat: integrate TrafficFSM, simplified controllers, updated interfaces)
  void laneCallback(
    const rusty_racer_interfaces::msg::LaneDeviation::SharedPtr msg);

  void trafficSignCallback(
    const rusty_racer_interfaces::msg::TrafficSign::SharedPtr msg);

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

<<<<<<< HEAD
  /**
   * @brief Helper to draw the steering arrow on the debug image
   */
=======
  // -- Debug overlay helper --------------------------------------------
>>>>>>> eabb7b2 (feat: integrate TrafficFSM, simplified controllers, updated interfaces)
  void drawControlOverlay(cv::Mat & img, double steering_angle, double velocity);

  // -- Controllers -----------------------------------------------------
  std::unique_ptr < LateralController > lateral_controller_;
  PIParams pi_params_;
  PIState pi_state_;

  // -- Traffic decision system -----------------------------------------
  TrafficFSM traffic_fsm_;
  TrafficParams traffic_params_;
  CamFrame latest_cam_frame_;
  rclcpp::Time last_traffic_update_;
  bool traffic_enabled_;

  // Timeout for FSM sign input [seconds].
  // Falls back to cruise speed when no TrafficSign received within this window.
  static constexpr double kTrafficSignTimeout = 2.5;

  // -- Target values ---------------------------------------------------
  double v_ref_;       // Cruise speed [m/s] (used when traffic_enabled_ == false,
                       // or as fallback on timeout)
  double y_target_;    // Lateral offset target [m]

  // -- Current state ---------------------------------------------------
  double current_v_;
  double current_psi_k_;
  rclcpp::Time last_update_time_;

  // -- Debug overlay image buffer --------------------------------------
  cv::Mat current_image_;

  // -- ROS2 communication ----------------------------------------------
  rclcpp::Subscription < nav_msgs::msg::Odometry > ::SharedPtr odom_sub_;
  rclcpp::Subscription < rusty_racer_interfaces::msg::LaneDeviation > ::SharedPtr lane_sub_;
  rclcpp::Subscription < rusty_racer_interfaces::msg::TrafficSign > ::SharedPtr traffic_sign_sub_;
  rclcpp::Subscription < sensor_msgs::msg::Image > ::SharedPtr sub_image_;
  rclcpp::Publisher < rusty_racer_interfaces::msg::MotorCommand > ::SharedPtr motor_cmd_pub_;
  rclcpp::Publisher < sensor_msgs::msg::Image > ::SharedPtr pub_debug_;
};

#endif  // RUSTY_RACER_CONTROL__CONTROL_NODE_H_
