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
 * @file control_node.cpp
 * @brief Rusty Racer control node implementation
 * @author zx
 * @date 2025-12
 */

// 1. Zugehöriger Header dieser Datei (muss an erster Stelle stehen)
#include "rusty_racer_control/control_node.h"

// 2. C++ System-Bibliotheken
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// 3. ROS 2 / Drittanbieter-Bibliotheken
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>    // NOLINT(build/include_order)
#include <tf2/LinearMath/Quaternion.h>   // NOLINT(build/include_order)
#include <tf2/utils.h>                   // NOLINT(build/include_order)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// 4. Andere Header dieses Projekts
// Interne Header des Control-Pakets
#include "rusty_racer_control/common.h"
#include "rusty_racer_control/laengsfuehrung_controller.h"
#include "rusty_racer_control/lateral_controller.h"
#include "rusty_racer_control/motor_mapping.h"
// Schnittstellen-Header (Interfaces)
#include "rusty_racer_interfaces/msg/lane_deviation.hpp"
#include "rusty_racer_interfaces/msg/motor_command.hpp"



/**
 * @class ControlNode
 * @brief Main control node for longitudinal and lateral control
 */
ControlNode::ControlNode()
: Node("control_node")
{
  // Vehicle parameters
  const double v_init = 0.5;
  const double l = 0.257;  // Wheelbase [m]
  const double l_h = 0.0;  // Sensor offset [m]

  // Lateral controller parameters
  const double lat_kp = 0.8;
  const double lat_kd = 0.2;

  // Initialize controllers
  lateral_controller_ =
    std::make_unique<LateralController>(v_init, l, l_h, lat_kp, lat_kd);

  // Longitudinal PI controller parameters
  PIParams pi_params;
  pi_params.Kp = 1.0;
  pi_params.Ki = 0.01;
  pi_params.v_min = 0.0;
  pi_params.v_max = 2.9;
  pi_params.a_lat_max = 1.5;   // Maximum lateral acceleration [m/s^2]
  pi_params.k_slowdown = 0.8;  // Lateral error slowdown coefficient

  pi_state_ = init_pi();
  pi_params_ = pi_params;

  // Target velocity and lateral offset
  v_ref_ = 0.8;     // Target velocity [m/s]
  y_target_ = -0.06;  // Target lateral offset [m]
                      // 0.0  = track center line
                      // 0.06 = maintain 60mm right offset
                      // -0.06 = maintain 60mm left offset

  // Initialize state
  current_v_ = 0.0;
  current_psi_k_ = 0.0;
  last_update_time_ = this->now();

  // Subscriber for odometry
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  // Subscriber for lane deviation
  lane_sub_ =
    this->create_subscription<rusty_racer_interfaces::msg::LaneDeviation>(
    "/lane_deviation", 10,
    std::bind(&ControlNode::laneCallback, this, std::placeholders::_1));

  // Publisher for motor commands
  motor_cmd_pub_ =
    this->create_publisher<rusty_racer_interfaces::msg::MotorCommand>(
    "/motor_command", 10);

  // Startup logging
  RCLCPP_INFO(this->get_logger(), "=================================");
  RCLCPP_INFO(this->get_logger(), "Control Node gestartet");
  RCLCPP_INFO(this->get_logger(), "=================================");
  RCLCPP_INFO(this->get_logger(), "Lateral: PD + Curvature Feedforward + y_target");
  RCLCPP_INFO(
    this->get_logger(),
    "Longitudinal: PI-Geschwindigkeitsregelung + Safe Velocity Planning");
  RCLCPP_INFO(this->get_logger(), "=================================");
  RCLCPP_INFO(this->get_logger(), "Lateral Parameter:");
  RCLCPP_INFO(this->get_logger(), "  Kp = %.2f", lat_kp);
  RCLCPP_INFO(this->get_logger(), "  Kd = %.2f", lat_kd);
  RCLCPP_INFO(this->get_logger(), "  y_target = %.3f m", y_target_);
  RCLCPP_INFO(this->get_logger(), "=================================");
  RCLCPP_INFO(this->get_logger(), "PI-Parameter:");
  RCLCPP_INFO(this->get_logger(), "  Kp = %.2f", pi_params.Kp);
  RCLCPP_INFO(this->get_logger(), "  Ki = %.2f", pi_params.Ki);
  RCLCPP_INFO(this->get_logger(), "  v_min = %.2f m/s", pi_params.v_min);
  RCLCPP_INFO(this->get_logger(), "  v_max = %.2f m/s", pi_params.v_max);
  RCLCPP_INFO(this->get_logger(), "  a_lat_max = %.2f m/s^2", pi_params.a_lat_max);
  RCLCPP_INFO(this->get_logger(), "  k_slowdown = %.2f", pi_params.k_slowdown);
  RCLCPP_INFO(this->get_logger(), "=================================");
  RCLCPP_INFO(this->get_logger(), "Sollgeschwindigkeit: %.2f m/s", v_ref_);
  RCLCPP_INFO(this->get_logger(), "=================================");
}

/**
 * @brief Odometry callback
 */
void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_v_ = msg->twist.twist.linear.x;
  current_psi_k_ = tf2::getYaw(msg->pose.pose.orientation);
  lateral_controller_->updateVelocity(current_v_);
  // Note: last_update_time_ is now only updated in laneCallback for consistent dt calculation
}

/**
 * @brief Lane deviation callback - main control loop
 */
void ControlNode::laneCallback(
  const rusty_racer_interfaces::msg::LaneDeviation::SharedPtr msg)
{
  // Extract data from LaneDeviation message
  double y = msg->lateral_error;
  double phi_k = msg->heading_error;
  double curvature = msg->curvature;  // Curvature for feedforward control

  // Calculate time step (using laneCallback timing for stable dt)
  auto current_time = this->now();
  double dt = (current_time - last_update_time_).seconds();
  if (dt <= 0.0 || dt > 0.1) {
    dt = 0.02;  // Default 50Hz if invalid
  }

  // Compute safe velocity based on curvature and lateral error
  double v_safe = compute_safe_velocity(pi_params_, v_ref_, curvature, std::abs(y));

  // Longitudinal control: PI controller + mapping
  double v_cmd = pi_step(pi_params_, pi_state_, v_safe, current_v_, dt);
  double motor_level = speed_to_motor_level(v_cmd, pi_params_.v_max);

  // Lateral control: PD controller with curvature feedforward
  double delta = lateral_controller_->compute(y, y_target_, phi_k, curvature, dt);

  // Create motor command message with timestamp
  auto cmd = rusty_racer_interfaces::msg::MotorCommand();
  cmd.header = msg->header;  // Timestamp from lane deviation message
  cmd.motor_level = motor_level;
  cmd.steering_angle = delta;
  motor_cmd_pub_->publish(cmd);

  // Update timestamp for next iteration
  last_update_time_ = current_time;
}

/**
 * @brief Main function
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
