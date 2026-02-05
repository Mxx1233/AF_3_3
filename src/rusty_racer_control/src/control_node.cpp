// Copyright 2025 Rusty Racer Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
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

// 1. Associated Header
#include "rusty_racer_control/control_node.h"

// 2. C++ System Libraries
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// 3. ROS 2 / Third-party Libraries
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
  const double lat_kp = 0.75;  // Original: 0.7
  const double lat_kd = 0.08;  // Original: 0.08

  // Initialize controllers
  lateral_controller_ =
    std::make_unique<LateralController>(v_init, l, l_h, lat_kp, lat_kd);

  // Longitudinal PI controller parameters
  PIParams pi_params;
  pi_params.Kp = 1.1;    // Original: 1.1
  pi_params.Ki = 0.015;  // Original: 0.015
  pi_params.v_min = 0.0;
  pi_params.v_max = 2.0; // Original: 2.0

  pi_state_ = init_pi();

  // Target velocity and lateral offset
  v_ref_ = 1.4;       // Target velocity [m/s] (Original: 0.8) (best time 1.4) actually 1.8 best
  y_target_ = -0.03;  // Target lateral offset [m] -0.03

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
  last_update_time_ = this->now();
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
  double y = msg->lateral_error;
  double phi_k = msg->heading_error;

  // Calculate time step
  auto current_time = this->now();
  double dt = (current_time - last_update_time_).seconds();
  if (dt <= 0.0 || dt > 0.1) {
    dt = 0.02;  // Default 50Hz if invalid
  }

  // Longitudinal control: PI controller + mapping
  double v_cmd = pi_step(pi_params_, pi_state_, v_ref_, current_v_, dt);
  double motor_level = speed_to_motor_level(v_cmd, pi_params_.v_max);

  // Lateral control: PD controller with y_target
  double delta = lateral_controller_->compute(y, y_target_, phi_k);

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