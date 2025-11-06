/**
 * @file firststeps_node.cpp
 * @brief implementation of the controller
 * @author PSAF
 * @date 2024-10-15
 */
#include "psaf_firststeps/firststeps_node.hpp"
#include <vector>
#include <string>
#include <iostream>

FirstStepsNode::FirstStepsNode()
: Node{"firststeps"}, test_seq_counter_{-1}
{
  const rclcpp::QoS qos{rclcpp::KeepLast(1)};

  publisher_motor_forward_ = this->create_publisher<std_msgs::msg::Int16>(
    SET_SPEED_FORWARD_TOPIC,
    qos);
  publisher_motor_backward_ = this->create_publisher<std_msgs::msg::Int16>(
    SET_SPEED_BACKWARD_TOPIC,
    qos);

  publisher_steering_ = this->create_publisher<std_msgs::msg::Int16>(SET_STEERING_TOPIC, qos);

  const std::vector<std::string> us_topics(US_TOPICS);

  for (size_t i = 0; i < us_topics.size(); ++i) {
    subscribers_us_.push_back(
      this->create_subscription<sensor_msgs::msg::Range>(
        us_topics[i],
        qos,
        [this, i](sensor_msgs::msg::Range::SharedPtr p) {this->updateUltrasonic(p, i);}));
  }

  subscriber_buttons_ = this->create_subscription<psaf_ucbridge_msgs::msg::Pbs>(
    GET_PUSHBUTTONS_TOPIC,
    qos,
    [this](psaf_ucbridge_msgs::msg::Pbs p) {this->updateButtons(p);});
}

void FirstStepsNode::updateUltrasonic(sensor_msgs::msg::Range::SharedPtr p, size_t i)
{
  if (i > 3) {
    RCLCPP_ERROR(this->get_logger(), "Unexpected ultrasonic sensor number: %ld", i);
    return;
  }

  us_values_[i] = p->range;
}

void FirstStepsNode::updateButtons(psaf_ucbridge_msgs::msg::Pbs p)
{
  button_values_ = {p.pba, p.pbb, p.pbc};
}

void FirstStepsNode::publishSpeed(int16_t speed, SpeedDir dir)
{
  // Publish either(!) using publish_motor_forward_ or publish_motor_backward_.
  // It makes no sense to use both in one cycle as the last message send just
  // overrules the ones send before.
  if (dir == SpeedDir::Forward) {
    std_msgs::msg::Int16 d;
    d.data = (speed > 1000) ? 1000 : ((speed < -500) ? -500 : speed);

    publisher_motor_forward_->publish(d);
  } else {
    std_msgs::msg::Int16 d;
    d.data = (speed > 500) ? 500 : ((speed < 0) ? 0 : speed);

    publisher_motor_backward_->publish(d);
  }
}

void FirstStepsNode::publishSteering(int16_t value)
{
  std_msgs::msg::Int16 d;
  d.data = (value > 300) ? 300 : ((value < -300) ? -300 : value);

  publisher_steering_->publish(d);
}

void FirstStepsNode::update()
{
  bool buttons_pressed[3] = {false, false, false};

  // Determine for each button if it was pressed since the last call to update.
  // For simplicity, we assume that a button was pressed if the new button value
  // is different to the old one, and the new button value is even
  // (thus the button is released again).
  // That is, if the button was pressed, released and then pressed again in one update
  // cycle, no button press is signaled until the button is released again.
  // (And one of the presses is "lost" in this case.)
  for (size_t i = 0; i < 3; ++i) {
    if (button_values_[i] == UNINIT_BUTTON) {
      continue;
    }

    if (prev_button_values_[i] == UNINIT_BUTTON) {
      prev_button_values_[i] = button_values_[i];
      continue;
    }

    if ((button_values_[i] != prev_button_values_[i]) && (button_values_[i] % 2 == 0)) {
      prev_button_values_[i] = button_values_[i];
      buttons_pressed[i] = true;
    }
  }

  if (TESTING) {
    this->testController(buttons_pressed[0]);
  }
}

void FirstStepsNode::testController(bool start_stop)
{
  // a value of -1 for test_seq_counter means that the cycle is stopped

  if (start_stop) {
    // if the start_stop command is given:
    //  - if the cycle is stopped, just start it
    //  - if the cycle runs, set the actutators to neutral positions
    //    and stop the cycle
    if (test_seq_counter_ == -1) {
      test_seq_counter_ = 0;
    } else {
      publishSpeed(0, SpeedDir::Forward);
      publishSteering(0);

      test_seq_counter_ = -1;
    }
  }

  // output 1 for each sensor whose reading is less than 10 cm (otherwise output 0)
  RCLCPP_INFO(
    get_logger(), "us warnings: l:%d f:%d r:%d",
    us_values_[2] < 0.1, us_values_[0] < 0.1, us_values_[1] < 0.1);

  if (test_seq_counter_ == -1) {
    RCLCPP_INFO(this->get_logger(), "Press A to start/stop test");
    return;
  }

  // Systemtest
  if (test_seq_counter_ == 0) {
    RCLCPP_INFO(this->get_logger(), "Systemtest");
    RCLCPP_INFO(this->get_logger(), "Motorlevel Test");
    RCLCPP_INFO(this->get_logger(), "Expect wheels to spin backwards, then forwards");

  } else if (test_seq_counter_ < 500) {
    const int speed = test_seq_counter_ * 3 - 500;

    if (speed < 0) {
      publishSpeed(-speed, SpeedDir::Backward);
    } else {
      publishSpeed(speed, SpeedDir::Forward);
    }
    RCLCPP_INFO(this->get_logger(), "Requested Speed: %d", speed);

  } else if (test_seq_counter_ == 500) {
    const int speed = 0;
    publishSpeed(speed, SpeedDir::Forward);
    RCLCPP_INFO(this->get_logger(), "Requested Speed: %d", speed);
    RCLCPP_INFO(
      this->get_logger(),
      "Finished Motorlevel Test. The wheels should have moved backwards, then forwards");

    RCLCPP_INFO(this->get_logger(), "Steering Test");
    RCLCPP_INFO(this->get_logger(), "Expect wheels to turn from left to right");

  } else if (test_seq_counter_ < 750) {
    const int steering_angle = (test_seq_counter_ - 500) * 2 - 250;

    publishSteering(steering_angle);
    RCLCPP_INFO(this->get_logger(), "Requested Steering: %d", steering_angle);

  } else {
    // set counter to -2 such that after incrementing at the end of this
    // method, it is -1 and the cycle stops.
    test_seq_counter_ = -2;

    const int steering_angle = 0;
    publishSteering(steering_angle);
    RCLCPP_INFO(this->get_logger(), "Requested Steering: %d", steering_angle);

    RCLCPP_INFO(this->get_logger(), "Finished Steering Test");
    RCLCPP_INFO(get_logger(), "Finished system test");
  }

  ++test_seq_counter_;
}
