#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "rusty_racer_interfaces/msg/motor_command.hpp"
#include "low_level_pkg/uc_mapper.hpp"

// =======================
// Einfache Konfiguration
// =======================

// Topics
#define MOTOR_COMMAND_TOPIC "/motor_command"
#define SET_STEERING_TOPIC  "/uc_bridge/set_steering"
#define SET_MOTOR_LEVEL_FORWARD_TOPIC  "/uc_bridge/set_motor_level_forward"
#define SET_MOTOR_LEVEL_BACKWARD_TOPIC  "/uc_bridge/set_motor_level_backward"

class UcBridgeAdapterNode : public rclcpp::Node
{
public:
  UcBridgeAdapterNode()
  : rclcpp::Node("uc_bridge_adapter_node")
  {
    UcMapper::Config config;
    mapper_ = std::make_unique<UcMapper>(config);

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

    // Publisher: Set-Motor-Level-Forward (Int16)
    set_motor_level_backward_pub_ = create_publisher<std_msgs::msg::Int16>(
      SET_MOTOR_LEVEL_BACKWARD_TOPIC, 10);


    RCLCPP_INFO(get_logger(), "uc_bridge_adapter_node gestartet");
  }

private:
    // MotorCommand-Callback: wandelt MotorCommand in Set-Steering und Set-Motor-Level-Forward um
  void motorCommandCallback(const rusty_racer_interfaces::msg::MotorCommand::SharedPtr msg)
  {
    // Kern--Mapping-Logik aufrufen
    UcMapper::CommandOutput output = mapper_->mapCommand(
      msg->steering_angle,
      msg->motor_level);

    // Lenkungsinformation veröffentlichen
    std_msgs::msg::Int16 steering_msg;
    steering_msg.data = output.steering;
    set_steering_pub_->publish(steering_msg);

    // Motorlevel veröffentlichen
    std_msgs::msg::Int16 motor_level_forward_msg;
    std_msgs::msg::Int16 motor_level_backward_msg;

    // motor_level_forward_msg.data = output.motor_fwd;
    // motor_level_backward_msg.data = output.motor_bwd;


    if (output.motor_fwd > 0) {
      motor_level_forward_msg.data = output.motor_fwd;
      set_motor_level_forward_pub_->publish(motor_level_forward_msg);

    } else if (output.motor_bwd > 0) {
      motor_level_backward_msg.data = output.motor_bwd;
      set_motor_level_backward_pub_->publish(motor_level_backward_msg);

    } else {
      motor_level_forward_msg.data = 0;
      motor_level_backward_msg.data = 0;
      set_motor_level_forward_pub_->publish(motor_level_forward_msg);
      set_motor_level_backward_pub_->publish(motor_level_backward_msg);
    }
  }

  std::unique_ptr<UcMapper> mapper_;

  // --- ROS I/O ---_forward_pub_->publish(motor_level_forward_msg);
  rclcpp::Subscription<rusty_racer_interfaces::msg::MotorCommand>::SharedPtr motor_command_sub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr set_steering_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr set_motor_level_forward_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr set_motor_level_backward_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UcBridgeAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
