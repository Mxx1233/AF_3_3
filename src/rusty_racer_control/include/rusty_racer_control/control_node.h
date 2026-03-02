#ifndef RUSTY_RACER_CONTROL__CONTROL_NODE_H_
#define RUSTY_RACER_CONTROL__CONTROL_NODE_H_

// C++ system headers
#include <memory>
#include <vector>

// ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>  // <--- NEW
#include <cv_bridge/cv_bridge.hpp>      // <--- NEW
#include <opencv2/opencv.hpp>         // <--- NEW

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
  ControlNode();

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Main control loop (triggered by Lane Deviation)
   */
  void laneCallback(
    const rusty_racer_interfaces::msg::LaneDeviation::SharedPtr msg);

  /**
   * @brief Stores the latest camera image for the overlay
   */
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief Helper to draw the steering arrow on the debug image
   */
  void drawControlOverlay(cv::Mat & img, double steering_angle, double velocity);

  // Controllers
  std::unique_ptr < LateralController > lateral_controller_;
  PIParams pi_params_;
  PIState pi_state_;

  // Target values
  double v_ref_;
  double y_target_;

  // Current state
  double current_v_;
  double current_psi_k_;
  rclcpp::Time last_update_time_;

  // Visualization State
  cv::Mat current_image_; // Stores the latest frame from camera

  // ROS2 communication
  rclcpp::Subscription < nav_msgs::msg::Odometry > ::SharedPtr odom_sub_;
  rclcpp::Subscription < rusty_racer_interfaces::msg::LaneDeviation > ::SharedPtr lane_sub_;
  rclcpp::Publisher < rusty_racer_interfaces::msg::MotorCommand > ::SharedPtr motor_cmd_pub_;

  // <--- NEW: Camera topics matching TrajectoryNode pattern
  rclcpp::Subscription < sensor_msgs::msg::Image > ::SharedPtr sub_image_;
  rclcpp::Publisher < sensor_msgs::msg::Image > ::SharedPtr pub_debug_;
};

#endif  // RUSTY_RACER_CONTROL__CONTROL_NODE_H_
