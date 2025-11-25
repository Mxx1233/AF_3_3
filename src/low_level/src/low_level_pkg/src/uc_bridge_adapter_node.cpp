#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "rusty_racer_interfaces/msg/motor_command.hpp"

// =======================
// Einfache Konfiguration
// =======================

// Lenkungparameter
#define SET_STEERING_LEFT_MAX  -250
#define SET_STEERING_CENTER     0
#define SET_EERING_RIGHT_MAX  250
#define ANGLE_STEERING_LEFT_MAX_RAD  -M_PI / 3
#define ANGLE_STEERING_CENTER_RAD     0
#define ANGLE_STEERING_RIGHT_MAX_RAD  M_PI / 3

// Motorlevel-Parameter
#define SET_MOTOR_LEVEL_FORWARD_MIN  -1000 // Bremsen (Wert nicht ganz sicher festgelegt)
#define SET_MOTOR_LEVEL_FORWARD_0        0   // Motor aus
#define SET_MOTOR_LEVEL_FORWARD_MAX   1000 // Vollgas vorwärts

// Topics
#define MOTOR_COMMAND_TOPIC "/motor_command"
#define SET_STEERING_TOPIC  "/uc_bridge/set_steering"
#define SET_MOTOR_LEVEL_FORWARD_TOPIC  "/uc_bridge/set_motor_level_forward"

class UcBridgeAdapterNode : public rclcpp::Node
{
public:
  UcBridgeAdapterNode()
  : rclcpp::Node("uc_bridge_adapter_node")
  {
    // Subscriber: MotorCommand (Lenkwinkel [rad], Motorlevel [-1.0..1.0])
    motor_command_sub_ = create_subscription<rusty_racer_interfaces::msg::MotorCommand>(
      MOTOR_COMMAND_TOPIC, 10,
      std::bind(&UcBridgeAdapterNode::motorCommandCallback, this, std::placeholders::_1));

    // Publisher: Set-Steering (Int16)
    set_steering_pub_ = create_publisher<std_msgs::msg::Int16>(
      SET_STEERING_TOPIC, 10);

    // Publisher: Set-Motor-Level-Forward (Int16)
    set_motor_level_forward_pub_ = create_publisher<std_msgs::msg::Int16>(
      SET_MOTOR_LEVEL_FORWARD_TOPIC, 10);

    RCLCPP_INFO(get_logger(), "uc_bridge_adapter_node gestartet");
  }

private:
    // MotorCommand-Callback: wandelt MotorCommand in Set-Steering und Set-Motor-Level-Forward um
  void motorCommandCallback(const rusty_racer_interfaces::msg::MotorCommand::SharedPtr msg)
  {
    // Lenkung umrechnen
    int16_t steering_value = 0;
    if (msg->steering_angle <= ANGLE_STEERING_LEFT_MAX_RAD) {
      steering_value = SET_STEERING_LEFT_MAX;
    } else if (msg->steering_angle >= ANGLE_STEERING_RIGHT_MAX_RAD) {
      steering_value = SET_EERING_RIGHT_MAX;
    } else {
      // Linear interpolieren
      steering_value = static_cast<int16_t>(
        SET_STEERING_CENTER +
        (msg->steering_angle - ANGLE_STEERING_CENTER_RAD) *
        (SET_EERING_RIGHT_MAX - SET_STEERING_CENTER) /
        (ANGLE_STEERING_RIGHT_MAX_RAD - ANGLE_STEERING_CENTER_RAD));
    }
    std_msgs::msg::Int16 steering_msg;
    steering_msg.data = steering_value;
    set_steering_pub_->publish(steering_msg);

    // Motorlevel umrechnen
    int16_t motor_level_value = 0;
    if (msg->motor_level <= -1.0f) {
      motor_level_value = SET_MOTOR_LEVEL_FORWARD_MIN;
    } else if (msg->motor_level >= 1.0f) {
      motor_level_value = SET_MOTOR_LEVEL_FORWARD_MAX;
    } else {
      motor_level_value = static_cast<int16_t>(
        msg->motor_level * (SET_MOTOR_LEVEL_FORWARD_MAX - SET_MOTOR_LEVEL_FORWARD_0));
    }
    std_msgs::msg::Int16 motor_level_msg;
    motor_level_msg.data = motor_level_value;
    set_motor_level_forward_pub_->publish(motor_level_msg);
  }

    // --- ROS I/O ---
  rclcpp::Subscription<rusty_racer_interfaces::msg::MotorCommand>::SharedPtr motor_command_sub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr set_steering_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr set_motor_level_forward_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UcBridgeAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
