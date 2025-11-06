/**
 * @file controller_node.hpp
 * @brief Controller for the psaf 1 cars
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_FIRSTSTEPS_NODE_HPP_
#define PSAF_FIRSTSTEPS_NODE_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/int16.hpp"

#include "psaf_ucbridge_msgs/msg/pbs.hpp"

#include "psaf_configuration/configuration.hpp"

#define TESTING true

enum class SpeedDir
{
  Forward,
  Backward,
};

constexpr uint16_t UNINIT_BUTTON = 0xFFFF;

/**
 * @class FirstSteps
 * @brief
 * @details
 */
class FirstStepsNode : public rclcpp::Node
{
private:
  // state for test cycle
  int test_seq_counter_ = -1;
  // previous button states for detecting presses
  std::array<uint16_t, 3> prev_button_values_ = {UNINIT_BUTTON, UNINIT_BUTTON, UNINIT_BUTTON};

  // sensor values read from subscribed topics
  std::array<double, 3> us_values_;
  std::array<uint16_t, 3> button_values_ = {UNINIT_BUTTON, UNINIT_BUTTON, UNINIT_BUTTON};

  // ROS topic publisher
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_motor_forward_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_motor_backward_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_steering_;

  // ROS topic subscriptions
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> subscribers_us_;
  rclcpp::Subscription<psaf_ucbridge_msgs::msg::Pbs>::SharedPtr subscriber_buttons_;

public:
  FirstStepsNode();

  /**
   * @brief Method in which the results get published
   * @details This method is called periodically by the main method of the node.
   */
  void update();

private:
  /**
   * @brief Method to publish new steering values
   * @details Valid value ranges [-300, 300]
   * Values outside this range are clamped to this range.
   * @param[in] value the new steering value to be set
   */
  void publishSteering(int16_t value);

  /**
   * @brief Callback method for the measured speed
   * @param[in] speed the new speed to be set
   * a positive value always means a demanded movement in the direction specified
   * by the parameter dir.
   * If dir == SpeedDir::Forward speed can be set to a negative value for active
   * breaking.
   * Valid value ranges:
   *  SpeedDir::Forward: [-500, 1000]
   *  SpeedDir::Backward: [0, 500]
   * Values outside these ranges are clamped to this range.
   * @param[in] dir direction of demanded driving
   */
  void publishSpeed(int16_t speed, SpeedDir dir);

  /**
   * @brief Callback method for ultrasonic senors values
   * @param[in] p the current sensor data
   * @param[in] i the id of the sensor (corresponds to the position in the list
   * US_TOPICS provided by the configuration)
   */
  void updateUltrasonic(sensor_msgs::msg::Range::SharedPtr p, size_t i);

  /**
   * @brief Callback method for button states
   * @param[in] p the current button states
   */
  void updateButtons(psaf_ucbridge_msgs::msg::Pbs p);

  /**
   * @brief (to be called in each cycle) test for steering and speed
   * @details This function tests the steering and speed control.
   * To activate it, set the TESTING flag at the top of the file to true.
   * This function can be used to check if the commands arrive at the uc_bridge
   * and to check if they are processed correctly.
   * The test cycle can be started and stopped by the parameter.
   * The initial state is stopped.
   * IMPORTANT: Make sure to place the car on a test stand and not on the road.
   * Make sure the wheels can rotate freely.
   * @param[in] start_stop toggle the cycle state
   */
  void testController(bool start_stop);
};

#endif  // PSAF_FIRSTSTEPS_NODE_HPP_
