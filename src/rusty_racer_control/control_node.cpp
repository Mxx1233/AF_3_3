// // Copyright 2025 Rusty Racer Team
// //
// // Licensed under the Apache License, Version 2.0 (the "License");
// // you may not use this file except in compliance with the License.
// // You may obtain a copy of the License at
// //
// //     http://www.apache.org/licenses/LICENSE-2.0
// //
// // Unless required by applicable law or agreed to in writing, software
// // distributed under the License is distributed on an "AS IS" BASIS,
// // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// // See the License for the specific language governing permissions and
// // limitations under the License.

// /**
//  * @file control_node.cpp
//  * @brief Rusty Racer control node implementation
//  * @author zx
//  * @date 2025-12
//  */

// // 1. Zugehöriger Header dieser Datei (muss an erster Stelle stehen)
// #include "rusty_racer_control/control_node.h"

// // 2. C++ System-Bibliotheken
// #include <cmath>
// #include <functional>
// #include <memory>
// #include <string>
// #include <vector>

// // 3. ROS 2 / Drittanbieter-Bibliotheken
// #include <nav_msgs/msg/odometry.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <tf2/LinearMath/Matrix3x3.h>    // NOLINT(build/include_order)
// #include <tf2/LinearMath/Quaternion.h>   // NOLINT(build/include_order)
// #include <tf2/utils.h>                   // NOLINT(build/include_order)
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// // 4. Andere Header dieses Projekts
// // Interne Header des Control-Pakets
// #include "rusty_racer_control/common.h"
// #include "rusty_racer_control/laengsfuehrung_controller.h"
// #include "rusty_racer_control/lateral_controller.h"
// #include "rusty_racer_control/motor_mapping.h"
// // Schnittstellen-Header (Interfaces)
// #include "rusty_racer_interfaces/msg/lane_deviation.hpp"
// #include "rusty_racer_interfaces/msg/motor_command.hpp"



// /**
//  * @class ControlNode
//  * @brief Main control node for longitudinal and lateral control
//  */
// ControlNode::ControlNode()
// : Node("control_node")
// {
//   // Vehicle parameters
//   const double v_init = 0.5;
//   const double l = 0.257;  // Wheelbase [m]
//   const double l_h = 0.0;  // Sensor offset [m]

//   // Lateral controller parameters
//   const double lat_kp = 0.75;
//   const double lat_kd = 0.1;

//   // Initialize controllers
//   lateral_controller_ =
//     std::make_unique<LateralController>(v_init, l, l_h, lat_kp, lat_kd);

//   // Longitudinal PI controller parameters
//   PIParams pi_params;
//   pi_params.Kp = 1.0;
//   pi_params.Ki = 0.01;
//   pi_params.v_min = 0.0;
//   pi_params.v_max = 2.9;
//   // pi_params.a_lat_max = 1.5;   // Maximum lateral acceleration [m/s^2]
//   // pi_params.k_slowdown = 0;  // Lateral error slowdown coefficient

//   pi_state_ = init_pi();
//   pi_params_ = pi_params;

//   // Target velocity and lateral offset
//   v_ref_ = 0.8;     // Target velocity [m/s]
//   y_target_ = -0.06;  // Target lateral offset [m]
//                       // 0.0  = track center line
//                       // 0.06 = maintain 60mm right offset
//                       // -0.06 = maintain 60mm left offset

//   // Initialize state
//   current_v_ = 0.0;
//   current_psi_k_ = 0.0;
//   last_update_time_ = this->now();

//   // Subscriber for odometry
//   odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//     "/odom", 10,
//     std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

//   // Subscriber for lane deviation
//   lane_sub_ =
//     this->create_subscription<rusty_racer_interfaces::msg::LaneDeviation>(
//     "/lane_deviation", 10,
//     std::bind(&ControlNode::laneCallback, this, std::placeholders::_1));

//   // Publisher for motor commands
//   motor_cmd_pub_ =
//     this->create_publisher<rusty_racer_interfaces::msg::MotorCommand>(
//     "/motor_command", 10);

//   // Startup logging
//   RCLCPP_INFO(this->get_logger(), "=================================");
//   RCLCPP_INFO(this->get_logger(), "Control Node gestartet");
//   RCLCPP_INFO(this->get_logger(), "=================================");
//   RCLCPP_INFO(this->get_logger(), "Lateral: PD + Curvature Feedforward + y_target");
//   RCLCPP_INFO(
//     this->get_logger(),
//     "Longitudinal: PI-Geschwindigkeitsregelung + Safe Velocity Planning");
//   RCLCPP_INFO(this->get_logger(), "=================================");
//   RCLCPP_INFO(this->get_logger(), "Lateral Parameter:");
//   RCLCPP_INFO(this->get_logger(), "  Kp = %.2f", lat_kp);
//   RCLCPP_INFO(this->get_logger(), "  Kd = %.2f", lat_kd);
//   RCLCPP_INFO(this->get_logger(), "  y_target = %.3f m", y_target_);
//   RCLCPP_INFO(this->get_logger(), "=================================");
//   RCLCPP_INFO(this->get_logger(), "PI-Parameter:");
//   RCLCPP_INFO(this->get_logger(), "  Kp = %.2f", pi_params.Kp);
//   RCLCPP_INFO(this->get_logger(), "  Ki = %.2f", pi_params.Ki);
//   RCLCPP_INFO(this->get_logger(), "  v_min = %.2f m/s", pi_params.v_min);
//   RCLCPP_INFO(this->get_logger(), "  v_max = %.2f m/s", pi_params.v_max);
//   // RCLCPP_INFO(this->get_logger(), "  a_lat_max = %.2f m/s^2", pi_params.a_lat_max);
//   // RCLCPP_INFO(this->get_logger(), "  k_slowdown = %.2f", pi_params.k_slowdown);
//   RCLCPP_INFO(this->get_logger(), "=================================");
//   RCLCPP_INFO(this->get_logger(), "Sollgeschwindigkeit: %.2f m/s", v_ref_);
//   RCLCPP_INFO(this->get_logger(), "=================================");
// }

// /**
//  * @brief Odometry callback
//  */
// void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
// {
//   current_v_ = msg->twist.twist.linear.x;
//   current_psi_k_ = tf2::getYaw(msg->pose.pose.orientation);
//   lateral_controller_->updateVelocity(current_v_);
//   // Note: last_update_time_ is now only updated in laneCallback for consistent dt calculation
// }

// /**
//  * @brief Lane deviation callback - main control loop
//  */
// void ControlNode::laneCallback(
//   const rusty_racer_interfaces::msg::LaneDeviation::SharedPtr msg)
// {
//   // Extract data from LaneDeviation message
//   double y_raw = msg->lateral_error;
//   double phi_raw = msg->heading_error;
//   double curv_raw = msg->curvature;  // Curvature for feedforward control

//   // Calculate time step (using laneCallback timing for stable dt)
//   auto current_time = this->now();
//   double dt = (current_time - last_update_time_).seconds();
//   if (dt <= 0.0 || dt > 0.1) {
//     dt = 0.02;  // Default 50Hz if invalid
//   }

//   double y     = std::clamp(y_raw,     -0.30, 0.30);  // 先保守一点：最多±30cm
//   double phi_k = std::clamp(phi_raw, -0.60, 0.60);  // 最多±0.6rad(≈34°)
//   double curvature = std::clamp(curv_raw, -2.0, 2.0); 

//   const bool hit_y_border   = (std::abs(y_raw)   > 0.30);
//   const bool hit_phi_border = (std::abs(phi_raw) > 0.60);
//   const bool hit_curv_border= (std::abs(curv_raw)> 2.0);  

//   bool suspicious_jump =false;
//   if(have_last_steer_){
//     const double delta_candidate =lateral_controller_->compute(y,y_target_,phi_k,curvature,dt);
//     const double steer_candidate = -delta_candidate;
//     if (std::abs(steer_candidate - last_steer_cmd_) > 0.25) {
//       suspicious_jump = true;
//     }
//   }

//   const bool bad_frame = hit_y_border || hit_phi_border || hit_curv_border || suspicious_jump;

//   // Compute safe velocity based on curvature and lateral error
//   // double v_safe = compute_safe_velocity(pi_params_, v_ref_, curvature, std::abs(y));
  

//   // Longitudinal control: PI controller + mapping
//   double v_cmd = pi_step(pi_params_, pi_state_, v_ref, current_v_, dt);//v_safe to v_ref
//   double motor_level = speed_to_motor_level(v_cmd, pi_params_.v_max);

//   // Lateral control: PD controller with curvature feedforward
//   //double delta = lateral_controller_->compute(y, y_target_, phi_k, curvature, dt);
//   //lateral control with bad-frame hold
//   double steer_cmd = 0.0;
//   double delta = 0.0; 
//   if (bad_frame && have_last_steer_) {
//     // hold last steering for one bad perception frame
//     steer_cmd = last_steer_cmd_ *0.8;
//   } else {
//     delta = lateral_controller_->compute(y, y_target_, phi_k, curvature, dt);
//     steer_cmd = -delta;                 // keep your sign convention
//     last_steer_cmd_ = steer_cmd;
//     have_last_steer_ = true;
//   }
//   // Create motor command message with timestamp
//   auto cmd = rusty_racer_interfaces::msg::MotorCommand();
//   cmd.header = msg->header;  // Timestamp from lane deviation message
//   cmd.motor_level = motor_level;
//   //cmd.steering_angle = -delta;
//   cmd.steering_angle = steer_cmd;
//   motor_cmd_pub_->publish(cmd);

//   // Update timestamp for next iteration
//   last_update_time_ = current_time;

//   RCLCPP_INFO_THROTTLE(
//     this->get_logger(), *this->get_clock(), 200,
//     "y_raw=%.3f phi_raw=%.3f curv_raw=%.4f | y=%.3f phi=%.3f curv=%.4f | dt=%.3f bad=%d steer=%.3f delta=%.3f",
//     y_raw, phi_raw, curv_raw,
//     y, phi_k, curvature,
//     dt, bad_frame ? 1 : 0, steer_cmd, delta);


// }

// /**
//  * @brief Main function
//  */
// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<ControlNode>());
//   rclcpp::shutdown();
//   return 0;
// }




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

// /**
//  * @file control_node.cpp
//  * @brief Rusty Racer control node implementation
//  * @author zx
//  * @date 2025-12
//  */

// // 1. Zugehöriger Header dieser Datei (muss an erster Stelle stehen)
// #include "rusty_racer_control/control_node.h"

// // 2. C++ System-Bibliotheken
// #include <cmath>
// #include <functional>
// #include <memory>
// #include <string>
// #include <vector>

// // 3. ROS 2 / Drittanbieter-Bibliotheken
// #include <nav_msgs/msg/odometry.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <tf2/LinearMath/Matrix3x3.h>    // NOLINT(build/include_order)
// #include <tf2/LinearMath/Quaternion.h>   // NOLINT(build/include_order)
// #include <tf2/utils.h>                   // NOLINT(build/include_order)
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// // 4. Andere Header dieses Projekts
// // Interne Header des Control-Pakets
// #include "rusty_racer_control/common.h"
// #include "rusty_racer_control/laengsfuehrung_controller.h"
// #include "rusty_racer_control/lateral_controller.h"
// #include "rusty_racer_control/motor_mapping.h"
// // Schnittstellen-Header (Interfaces)
// #include "rusty_racer_interfaces/msg/lane_deviation.hpp"
// #include "rusty_racer_interfaces/msg/motor_command.hpp"

// /**
//  * @class ControlNode
//  * @brief Main control node for longitudinal and lateral control
//  */
// ControlNode::ControlNode()
// : Node("control_node")
// {
//   // Vehicle parameters
//   const double v_init = 0.5;
//   const double l = 0.257;  // Wheelbase [m]
//   const double l_h = 0.0;  // Sensor offset [m]

//   // Lateral controller parameters
//   const double lat_kp = 0.5;
//   const double lat_kd = 0.01;

//   // Initialize controllers
//   lateral_controller_ =
//     std::make_unique<LateralController>(v_init, l, l_h, lat_kp, lat_kd);

//   // Longitudinal PI controller parameters
//   PIParams pi_params;
//   pi_params.Kp = 1.0;
//   pi_params.Ki = 0.01;
//   pi_params.v_min = 0.0;
//   pi_params.v_max = 2.9;

//   pi_state_ = init_pi();

//   // Target velocity and lateral offset
//   v_ref_ = 0.3;     // Target velocity [m/s]
//   y_target_ = -0.06;  // Target lateral offset [m]
//                       // 0.0  = track center line
//                       // 0.06 = maintain 60mm right offset
//                       // -0.06 = maintain 60mm left offset

//   // Initialize state
//   current_v_ = 0.0;
//   current_psi_k_ = 0.0;
//   last_update_time_ = this->now();

//   // Subscriber for odometry
//   odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//     "/odom", 10,
//     std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

//   // Subscriber for lane deviation
//   lane_sub_ =
//     this->create_subscription<rusty_racer_interfaces::msg::LaneDeviation>(
//     "/lane_deviation", 10,
//     std::bind(&ControlNode::laneCallback, this, std::placeholders::_1));

//   // Publisher for motor commands
//   motor_cmd_pub_ =
//     this->create_publisher<rusty_racer_interfaces::msg::MotorCommand>(
//     "/motor_command", 10);

//   // Startup logging
//   RCLCPP_INFO(this->get_logger(), "=================================");
//   RCLCPP_INFO(this->get_logger(), "Control Node gestartet");
//   RCLCPP_INFO(this->get_logger(), "=================================");
//   RCLCPP_INFO(this->get_logger(), "Lateral: PD + Vorfilter + y_target");
//   RCLCPP_INFO(
//     this->get_logger(),
//     "Longitudinal: PI-Geschwindigkeitsregelung + Mapping");
//   RCLCPP_INFO(this->get_logger(), "=================================");
//   RCLCPP_INFO(this->get_logger(), "Lateral Parameter:");
//   RCLCPP_INFO(this->get_logger(), "  Kp = %.2f", lat_kp);
//   RCLCPP_INFO(this->get_logger(), "  Kd = %.2f", lat_kd);
//   RCLCPP_INFO(this->get_logger(), "  y_target = %.3f m", y_target_);
//   RCLCPP_INFO(this->get_logger(), "=================================");
//   RCLCPP_INFO(this->get_logger(), "PI-Parameter:");
//   RCLCPP_INFO(this->get_logger(), "  Kp = %.2f", pi_params.Kp);
//   RCLCPP_INFO(this->get_logger(), "  Ki = %.2f", pi_params.Ki);
//   RCLCPP_INFO(this->get_logger(), "  v_min = %.2f m/s", pi_params.v_min);
//   RCLCPP_INFO(this->get_logger(), "  v_max = %.2f m/s", pi_params.v_max);
//   RCLCPP_INFO(this->get_logger(), "=================================");
//   RCLCPP_INFO(this->get_logger(), "Sollgeschwindigkeit: %.2f m/s", v_ref_);
//   RCLCPP_INFO(this->get_logger(), "=================================");
// }

// /**
//  * @brief Odometry callback
//  */
// void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
// {
//   current_v_ = msg->twist.twist.linear.x;
//   current_psi_k_ = tf2::getYaw(msg->pose.pose.orientation);
//   lateral_controller_->updateVelocity(current_v_);
//   last_update_time_ = this->now();
// }

// /**
//  * @brief Lane deviation callback - main control loop
//  */
// void ControlNode::laneCallback(
//   const rusty_racer_interfaces::msg::LaneDeviation::SharedPtr msg)
// {
//   // Extract data from LaneDeviation message
//   double y = msg->lateral_error;
//   double phi_k = msg->heading_error;
//   // double curvature = msg->curvature;  // Optional parameter

//   // Calculate time step
//   auto current_time = this->now();
//   double dt = (current_time - last_update_time_).seconds();
//   if (dt <= 0.0 || dt > 0.1) {
//     dt = 0.02;  // Default 50Hz if invalid
//   }

//   // Longitudinal control: PI controller + mapping
//   double v_cmd = pi_step(pi_params_, pi_state_, v_ref_, current_v_, dt);
//   // double v_cmd = 0.5;  // Alternative: fixed speed
//   double motor_level = speed_to_motor_level(v_cmd, pi_params_.v_max);

//   // Lateral control: PD controller with y_target
//   double delta = lateral_controller_->compute(y, y_target_, phi_k);

//   // Create motor command message with timestamp
//   auto cmd = rusty_racer_interfaces::msg::MotorCommand();
//   cmd.header = msg->header;  // Timestamp from lane deviation message
//   cmd.motor_level = motor_level;
//   cmd.steering_angle = -delta;
//   motor_cmd_pub_->publish(cmd);
// }

// /**
//  * @brief Main function
//  */
// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<ControlNode>());
//   rclcpp::shutdown();
//   return 0;
// }
#include "rusty_racer_control/control_node.h"

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

// --- TF2 Includes (Order is important for linking!) ---
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
// -----------------------------------------------------

#include <sensor_msgs/image_encodings.hpp>

// 4. Project Headers
#include "rusty_racer_control/common.h"
#include "rusty_racer_control/laengsfuehrung_controller.h"
#include "rusty_racer_control/lateral_controller.h"
#include "rusty_racer_control/motor_mapping.h"
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
  const double v_init = 0.01;
  const double l = 0.257;  // Wheelbase [m]
  const double l_h = 0.0;  // Sensor offset [m]

  // Lateral controller parameters (reduced for low-speed gentle steering)
  const double lat_kp = 0.75;  // Original: 1.5  0.75  0.70  0.65
  const double lat_kd = 0.15;  // Original: 0.3  0.10  0.12  0.12

  // Initialize controllers
  lateral_controller_ =
    std::make_unique<LateralController>(v_init, l, l_h, lat_kp, lat_kd);

  // Longitudinal PI controller parameters
  PIParams pi_params;
  pi_params.Kp = 1.5;    // Original: 1.0
  pi_params.Ki = 0.015;  // Original: 0.01
  pi_params.v_min = 0.0;
  pi_params.v_max = 2.0; // Original: 2.9

  pi_state_ = init_pi();

  // Target velocity and lateral offset
  v_ref_ = 1.3;       // Target velocity [m/s] (Original: 0.8) (best time 1.4)
  y_target_ = -0.06;  // Target lateral offset [m]

  // Initialize state
  current_v_ = 0.0;
  current_psi_k_ = 0.0;
  last_update_time_ = this->now();

  // 1. Subscriber for odometry
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  // 2. Subscriber for lane deviation
  lane_sub_ =
    this->create_subscription<rusty_racer_interfaces::msg::LaneDeviation>(
    "/lane_deviation", 10,
    std::bind(&ControlNode::laneCallback, this, std::placeholders::_1));

  // 3. Subscriber for Camera (Visualization)
  // Matching the topic from TrajectoryNode
  sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/camera/color/image_raw", 10,
    std::bind(&ControlNode::imageCallback, this, std::placeholders::_1));

  // 4. Publisher for motor commands
  motor_cmd_pub_ =
    this->create_publisher<rusty_racer_interfaces::msg::MotorCommand>(
    "/motor_command", 10);

  // 5. Publisher for Debug Overlay
  pub_debug_ = this->create_publisher<sensor_msgs::msg::Image>(
    "control/debug_overlay", 10);

  // Startup logging
  RCLCPP_INFO(this->get_logger(), "=================================");
  RCLCPP_INFO(this->get_logger(), "Control Node Started");
  RCLCPP_INFO(this->get_logger(), "Visualization Topic: control/debug_overlay");
  RCLCPP_INFO(this->get_logger(), "=================================");
}

/**
 * @brief Odometry callback
 */
void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_v_ = msg->twist.twist.linear.x;

  // --- FIX: Explicit conversion to avoid linker error ---
  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.pose.orientation, q);
  
  // Convert quaternion to yaw (safe method)
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  current_psi_k_ = yaw;
  // ----------------------------------------------------

  lateral_controller_->updateVelocity(current_v_);
  //last_update_time_ = this->now();
}

/**
 * @brief Image callback - buffers the image for the main loop
 */
void ControlNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    // Store image for processing in the control loop
    // Using toCvCopy ensures we have a mutable copy
    current_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

/**
 * @brief Lane deviation callback - main control loop
 */
void ControlNode::laneCallback(
  const rusty_racer_interfaces::msg::LaneDeviation::SharedPtr msg)
{
  // Extract data from LaneDeviation message
  double y_raw = msg->lateral_error;
  double phi_raw = msg->heading_error;

  // Calculate time step
  auto current_time = this->now();
  double dt = (current_time - last_update_time_).seconds();
  if (dt <= 0.0 || dt > 0.1) {
    dt = 0.02;  // Default 50Hz if invalid
  }

    // ===================== 3) 输入限幅 + 可选低通滤波（强烈建议） =====================
  // (A) 先 clamp，避免偶发离谱值直接把控制打爆
  double y   = std::clamp(y_raw,   -0.30, 0.30);   // 依你赛道宽度可调
  double phi_k = std::clamp(phi_raw, -0.80, 0.80);   // rad

  // (B) 低通滤波：抑制抖动/虚线导致的跳变（alpha 越小越平滑）
  static bool filter_init = false;
  static double y_f = 0.0;
  static double phi_f = 0.0;
  const double alpha = 0.25;

  if (!filter_init) {
    y_f = y;
    phi_f = phi_k;
    filter_init = true;
  } else {
    y_f   = alpha * y   + (1.0 - alpha) * y_f;
    phi_f = alpha * phi_k + (1.0 - alpha) * phi_f;
  }

  // Longitudinal control: PI controller + mapping
  double v_cmd = pi_step(pi_params_, pi_state_, v_ref_, current_v_, dt);
  double motor_level = speed_to_motor_level(v_cmd, pi_params_.v_max);

    // ===================== 5) 横向控制（PD） + 异常保护 =====================
  static double last_delta = 0.0;

  // Lateral control: PD controller with y_target
  double delta = lateral_controller_->compute(y, y_target_, phi_k);

  // 异常保护：如果输入突然很怪，就不要把舵角“回正导致直走”
  // 你可以根据实际情况调阈值
  const bool bad_input =
    (std::abs(y_raw) > 0.35) || (std::abs(phi_raw) > 1.0) ||
    (!std::isfinite(y_raw)) || (!std::isfinite(phi_raw));

  if (bad_input) {
    delta = last_delta;   // 保持上一帧舵角（比回正直走安全）
    // 可选：丢线时降速，避免冲出跑道（不想降速就注释掉）
    // v_ref_ = 1.2;
  } else {
    // v_ref_ = 1.6; // 如果你上面用了降速，这里恢复
  }

  last_delta = delta;

  // ===================== 6) 打印 y/phi/delta（你要看的就是这个） =====================
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    200,  // 每 200ms 打一次
    "CTRL y=%.3f phi=%.3f | y_f=%.3f phi_f=%.3f | delta=%.3f | v=%.2f v_cmd=%.2f",
    y_raw, phi_raw, y_f, phi_f, delta, current_v_, v_cmd);
    // ===================== 7) 可视化 overlay（保持你原来的逻辑） =====================
  if (!current_image_.empty()) {
    cv::Mat debug_view = current_image_.clone();
    drawControlOverlay(debug_view, delta, v_cmd);

    auto out_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug_view).toImageMsg();
    out_msg->header.stamp = this->now();
    out_msg->header.frame_id = "camera_link";
    pub_debug_->publish(*out_msg);
  }

  // ===================== 8) 发布 motor command =====================
  rusty_racer_interfaces::msg::MotorCommand cmd;
  cmd.header = msg->header;
  cmd.motor_level = motor_level;
  cmd.steering_angle = -delta;   // 保持你现在的坐标系约定
  motor_cmd_pub_->publish(cmd);

  // ===================== 9) 最关键：更新 last_update_time_（必须在这里） =====================
  last_update_time_ = current_time;
}
  /*原来的
  // --- Visualization Logic ---
  if (!current_image_.empty()) {
    cv::Mat debug_view = current_image_.clone();
    
    // Draw the steering overlay
    drawControlOverlay(debug_view, delta, v_cmd);

    // Convert back to ROS message and publish
    sensor_msgs::msg::Image::SharedPtr out_msg = 
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug_view).toImageMsg();
    
    out_msg->header.stamp = this->now();
    out_msg->header.frame_id = "camera_link"; // Adjust if needed
    pub_debug_->publish(*out_msg);
  }

  // Create motor command message
  auto cmd = rusty_racer_interfaces::msg::MotorCommand();
  cmd.header = msg->header;
  cmd.motor_level = motor_level;
  cmd.steering_angle = -delta;  // Direct pass-through (coordinate system already corrected in trajectory)
  motor_cmd_pub_->publish(cmd);
}
*/
/**
 * @brief Helper to draw steering arrow and text
 */
void ControlNode::drawControlOverlay(cv::Mat& img, double steering_angle, double velocity)
{
  int h = img.rows;
  int w = img.cols;
  cv::Point center_bottom(w / 2, h);

  // 1. Draw Steering Arrow
  // Length relative to image height
  double arrow_len = h * 0.35; 
  
  // Calculate tip position
  // delta > 0 is Left Turn. In Image X grows right, so Left is -X.
  // We use sin/cos logic:
  // x = start_x - len * sin(angle)
  // y = start_y - len * cos(angle)
  cv::Point arrow_tip;
  arrow_tip.x = center_bottom.x - static_cast<int>(arrow_len * std::sin(steering_angle));
  arrow_tip.y = center_bottom.y - static_cast<int>(arrow_len * std::cos(steering_angle));

  // Cyan Color (BGR: 255, 255, 0)
  cv::Scalar color(255, 255, 0); 
  cv::arrowedLine(img, center_bottom, arrow_tip, color, 5, 8, 0, 0.1);

  // 2. Draw Text Info with background box
  std::string txt_steer = "Steer: " + std::to_string(steering_angle);
  std::string txt_vel   = "Vel Ref: " + std::to_string(velocity);
  
  // Background box (Top Left)
  cv::rectangle(img, cv::Point(0, 0), cv::Point(250, 80), cv::Scalar(0,0,0), -1);
  
  // Text
  cv::putText(img, txt_steer, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);
  cv::putText(img, txt_vel,   cv::Point(10, 65), cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);
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
