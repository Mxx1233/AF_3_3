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

#include "rusty_racer_control/control_node.h"

#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>

// TF2 includes
#include <tf2/LinearMath/Matrix3x3.h>  // NOLINT(build/include_order)
#include <tf2/LinearMath/Quaternion.h>  // NOLINT(build/include_order)
#include <tf2/utils.h>  // NOLINT(build/include_order)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rusty_racer_control/common.h"
#include "rusty_racer_control/laengsfuehrung_controller.h"
#include "rusty_racer_control/lateral_controller.h"
#include "rusty_racer_control/motor_mapping.h"
#include "rusty_racer_interfaces/msg/lane_deviation.hpp"
#include "rusty_racer_interfaces/msg/motor_command.hpp"
#include "rusty_racer_interfaces/msg/traffic_sign.hpp"

// =============================================================================
//  Constructor
// =============================================================================

ControlNode::ControlNode()
: Node("control_node")
{
  // -- Vehicle parameters ------------------------------------------------
  const double v_init = 0.01;
  const double l = 0.257;        // Wheelbase [m]
  const double l_h = 0.0;        // Sensor offset [m]

  // Lateral controller gains (gentle steering for low speed)
  const double lat_kp = 0.75;
  const double lat_kd = 0.15;

  lateral_controller_ =
    std::make_unique<LateralController>(v_init, l, l_h, lat_kp, lat_kd);

  // -- Longitudinal PI parameters ----------------------------------------
  pi_params_.Kp = 0.75;
  pi_params_.Ki = 0.03;
  pi_params_.v_min = 0.0;
  pi_params_.v_max = 2.0;

  pi_state_ = init_pi();

  // -- Traffic FSM parameters --------------------------------------------
  traffic_params_.conf_th = 0.0f;
  traffic_params_.v_default = 1.15f;
  traffic_params_.v_speed30 = 0.75f;
  traffic_params_.v_highway = 1.4f;
  traffic_params_.v_yield = 0.3f;
  traffic_params_.d_stop_trigger = 1.0f;
  traffic_params_.d_yield_trigger = 2.0f;
  traffic_params_.d_release = 3.0f;
  traffic_params_.stop_hold_ms = 3000;
  traffic_params_.on_count = 3;
  traffic_params_.off_count = 3;
  traffic_params_.start_on_count = 3;
  traffic_params_.start_off_count = 6;
  traffic_params_.start_stop_lock_dist_m = 0.30f;
  traffic_params_.same_sign_block_dist_m = 1.0f;

  // Layer 1 speed mode transition delays -- adjust here for tuning
  // Formula: delay_ms ≈ detection_dist(~2m) / approach_speed(m/s) * 1000
  traffic_params_.s30_start_delay_ms = 1500;  // entering Speed30 (approach ~1.0 m/s)
  traffic_params_.s30_end_delay_ms = 3000;    // leaving  Speed30 (approach ~0.5 m/s)
  traffic_params_.hw_start_delay_ms = 1500;   // entering Highway (approach ~1.0 m/s)
  traffic_params_.hw_end_delay_ms = 1000;     // leaving  Highway (approach ~1.5 m/s)

  traffic_fsm_.reset();
  last_traffic_update_ = this->now();

  // -- Mode selection ----------------------------------------------------
  // true = gate-only; false = full FSM
  cruise_mode_ = this->declare_parameter("cruise_mode", true);
  cruise_gate_unlocked_ = false;
  cruise_stop_seen_ = false;
  cruise_stop_on_count_ = 0;
  cruise_stop_gone_count_ = 0;

  // -- Target values ---------------------------------------------------
  v_ref_ = this->declare_parameter<double>("v_ref", 1.15);  // V7: dynamic param
  y_target_ = -0.06;   // Lateral offset [m] (negative = left bias)

  // -- Initial state -----------------------------------------------------
  current_v_ = 0.0;
  current_psi_k_ = 0.0;
  last_update_time_ = this->now();

  // -- Subscribers -------------------------------------------------------
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  lane_sub_ = this->create_subscription<rusty_racer_interfaces::msg::LaneDeviation>(
    "/lane_deviation", 10,
    std::bind(&ControlNode::laneCallback, this, std::placeholders::_1));

  traffic_sign_sub_ = this->create_subscription<rusty_racer_interfaces::msg::TrafficSign>(
    "/traffic_sign/detections", 10,
    std::bind(&ControlNode::trafficSignCallback, this, std::placeholders::_1));

  // -- Publishers --------------------------------------------------------
  motor_cmd_pub_ = this->create_publisher<rusty_racer_interfaces::msg::MotorCommand>(
    "/motor_command", 10);

  this->declare_parameter("topics.state_info_topic", "/state_info");
  this->declare_parameter("topics.target_v_topic", "/target_velocity");
  
  std::string state_t = this->get_parameter("topics.state_info_topic").as_string();
  std::string v_t = this->get_parameter("topics.target_v_topic").as_string();

  state_gate_pub_   = this->create_publisher<std_msgs::msg::String>("/fsm/gate", 10);
  state_mode_pub_   = this->create_publisher<std_msgs::msg::String>("/fsm/mode", 10);
  state_action_pub_ = this->create_publisher<std_msgs::msg::String>("/fsm/action", 10);

  target_v_pub_ = this->create_publisher<std_msgs::msg::Float32>(v_t, 10);
  // -- Startup log -------------------------------------------------------
  RCLCPP_INFO(this->get_logger(), "=================================");
  RCLCPP_INFO(this->get_logger(), "Control Node Started");
  RCLCPP_INFO(this->get_logger(), "Lateral:      PD (no curvature feedforward)");
  RCLCPP_INFO(this->get_logger(), "Longitudinal: PI + Motor Mapping");
  RCLCPP_INFO(
    this->get_logger(), "Mode:         %s",
    cruise_mode_ ? "CRUISE (gate-only)" : "FULL FSM");
  RCLCPP_INFO(this->get_logger(), "=================================");
  RCLCPP_INFO(
    this->get_logger(), "Lateral  Kp=%.2f  Kd=%.2f  y_target=%.3fm",
    lat_kp, lat_kd, y_target_);
  RCLCPP_INFO(
    this->get_logger(), "PI  Kp=%.2f  Ki=%.4f  v_max=%.2fm/s",
    pi_params_.Kp, pi_params_.Ki, pi_params_.v_max);
  RCLCPP_INFO(this->get_logger(), "v_ref=%.2f m/s", v_ref_);
  RCLCPP_INFO(this->get_logger(), "=================================");
}

// =============================================================================
//  Odometry callback
// =============================================================================

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const double raw_v = msg->twist.twist.linear.x;
  const double alpha = 0.5;  // Low-pass filter coefficient (0=no update, 1=no filter)
  current_v_ = alpha * raw_v + (1.0 - alpha) * current_v_;

  // Explicit quaternion-to-yaw conversion (avoids tf2::getYaw linker issue)
  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.pose.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  current_psi_k_ = yaw;

  lateral_controller_->updateVelocity(current_v_);
}

// =============================================================================
//  Traffic sign callback (async, updates cached CamFrame)
// =============================================================================

void ControlNode::trafficSignCallback(
  const rusty_racer_interfaces::msg::TrafficSign::SharedPtr msg)
{
  latest_cam_frame_ = toCamFrame(*msg, traffic_params_);
  last_traffic_update_ = this->now();
}

// =============================================================================
//  Lane deviation callback (main control loop, ~50 Hz)
// =============================================================================

void ControlNode::laneCallback(
  const rusty_racer_interfaces::msg::LaneDeviation::SharedPtr msg)
{
  const double y = msg->lateral_error;
  const double phi_k = msg->heading_error;

  // -- Time step ---------------------------------------------------------
  auto current_time = this->now();
  double dt = (current_time - last_update_time_).seconds();
  if (dt <= 0.0 || dt > 0.1) {
    dt = 0.02;  // Default 50 Hz
  }

  // -- V7: Poll dynamic speed parameter (runtime: ros2 param set /control_node v_ref X.XX)
  v_ref_ = this->get_parameter("v_ref").as_double();
  traffic_params_.v_default = static_cast<float>(v_ref_);  // keep FSM default in sync

  // -- Determine velocity reference --------------------------------------
  double v_target = v_ref_;         // Default: fixed cruise speed
  bool emergency_stop = false;

  if (cruise_mode_) {
    // ── Cruise path: gate-only ──────────────────────────────────────────
    static double v_ramp = 0.0;  // Shared ramp state across both branches
    if (!cruise_gate_unlocked_) {
      // Two-phase start gate
      bool has_stop = false;
      for (const auto & d : latest_cam_frame_.dets) {
        if (d.valid && d.type == SignType::Stop) {has_stop = true; break;}
      }
      if (!cruise_stop_seen_) {
        // Phase 1: confirm Stop sign is present
        if (has_stop) {
          cruise_stop_on_count_++;
        } else {
          cruise_stop_on_count_ = 0;
        }
        if (cruise_stop_on_count_ >= traffic_params_.start_on_count) {
          cruise_stop_seen_ = true;
          cruise_stop_gone_count_ = 0;
          RCLCPP_INFO(this->get_logger(), "CRUISE: Stop sign confirmed, waiting for removal");
        }
      } else {
        // Phase 2: wait for Stop sign to disappear
        if (!has_stop) {
          cruise_stop_gone_count_++;
        } else {
          cruise_stop_gone_count_ = 0;
        }
        if (cruise_stop_gone_count_ >= traffic_params_.start_off_count) {
          cruise_gate_unlocked_ = true;
          pi_state_.v_cmd = 0.0;     // Start ramp from 0
          pi_state_.e_pre = 0.0;
          pi_state_.integral = 0.0;  // V6: reset position-form integral
          RCLCPP_INFO(this->get_logger(), "CRUISE: Gate cleared -> ramping up");
        }
      }
      v_ramp = 1.0;  // Reset ramp while gate is locked
      v_target = 0.0;
      emergency_stop = true;
    } else {
      // Gate cleared: ramp v_target from 0 to v_ref_ at 0.3 m/s²
      v_ramp = std::min(v_ramp + 0.3 * dt, v_ref_);
      v_target = v_ramp;
    }

  } else {
    // ── FSM path: full three-layer logic ────────────────────────────────
    double sign_age = (current_time - last_traffic_update_).seconds();

    uint32_t dt_ms = static_cast<uint32_t>(dt * 1000.0);
    if (dt_ms == 0) {
      dt_ms = 20;
    }

    CamFrame fsm_input =
      (sign_age <= kTrafficSignTimeout) ? latest_cam_frame_ : CamFrame{};

    if (sign_age > kTrafficSignTimeout) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "TrafficSign timeout (%.1fs). FSM running on empty frame.",
        sign_age);
    }

    DecisionOut decision = traffic_fsm_.step(fsm_input, dt_ms, traffic_params_);

    v_target = decision.v_ref_mps;
    emergency_stop = decision.must_stop;
  }

  // Emergency stop overrides everything
  if (emergency_stop) {
    v_target = 0.0;
  }

  // -- V7: Longitudinal control: Open-loop feedforward only (no PI) ------
  // To restore PI: see Pre-V7 snapshot in speed_control_versions.md
  double motor_level = std::max(0.0, std::min(v_target / pi_params_.v_max, 1.0));

  // -- Lateral control: 3-param PD (no curvature feedforward) ------------
  double delta = lateral_controller_->compute(y, y_target_, phi_k);

  // -- Publish motor command ---------------------------------------------
  auto cmd = rusty_racer_interfaces::msg::MotorCommand();
  cmd.header.stamp = msg->header.stamp;
  cmd.motor_level = motor_level;
  // Negate delta: coordinate system correction (trajectory already inverted)
  cmd.steering_angle = -delta;
  motor_cmd_pub_->publish(cmd);

  last_update_time_ = current_time;

  // -- Publish Debug Info -------------------------------------------------
  auto fsm_info = traffic_fsm_.getDebugInfo();
  
  std::string gate_s = (fsm_info.layer0_gate == 2) ? "OPEN" : (fsm_info.layer0_gate == 1 ? "CONFIRMING" : "LOCKED");
  std::string mode_s = (fsm_info.layer1_mode == 1) ? "SPEED_30" : (fsm_info.layer1_mode == 2 ? "HIGHWAY" : "DEFAULT");
  std::string act_s  = (fsm_info.layer2_action == 1) ? "STOPPING" : (fsm_info.layer2_action == 2 ? "YIELDING" : "NONE");

  auto msg_g = std_msgs::msg::String(); msg_g.data = gate_s; state_gate_pub_->publish(msg_g);
  auto msg_m = std_msgs::msg::String(); msg_m.data = mode_s; state_mode_pub_->publish(msg_m);
  auto msg_a = std_msgs::msg::String(); msg_a.data = act_s;  state_action_pub_->publish(msg_a);

  auto v_msg = std_msgs::msg::Float32();
  v_msg.data = static_cast<float>(v_target);
  target_v_pub_->publish(v_msg);
}

// =============================================================================
//  Entry point
// =============================================================================

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
