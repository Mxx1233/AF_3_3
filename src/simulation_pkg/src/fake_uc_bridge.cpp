#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"

using namespace std::chrono_literals;

class FakeUcBridge : public rclcpp::Node
{
public:
  FakeUcBridge()
  : Node("fake_uc_bridge")
  {
        // Publishers to simulate sensors
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_data", 10);
    dt_pub_ = this->create_publisher<std_msgs::msg::Float32>("/dt_data", 10);

    timer_ = this->create_wall_timer(50ms, std::bind(&FakeUcBridge::timer_callback, this));

        // Subscribers to check control commands
    steer_sub_ = this->create_subscription<std_msgs::msg::Int16>(
            "/uc_bridge/set_steering", 10,
            std::bind(&FakeUcBridge::steering_cb, this, std::placeholders::_1));

    motor_sub_ = this->create_subscription<std_msgs::msg::Int16>(
            "/uc_bridge/set_motor_level_forward", 10,
            std::bind(&FakeUcBridge::motor_cb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Fake uc_bridge started. Publishing simulated sensor data.");
  }

private:
  void timer_callback()
  {
        // Simulate moving straight at a constant speed
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.angular_velocity.z = 0.0;     // No rotation
    imu_pub_->publish(imu_msg);

        // Simulate speed of 0.5 m/s. From formula: v = (1/8 * 0.22) / dt
        // So, dt = (1/8 * 0.22) / 0.5 = 0.055
    auto dt_msg = std_msgs::msg::Float32();
    dt_msg.data = 0.055f;
    dt_pub_->publish(dt_msg);
  }

  void steering_cb(const std_msgs::msg::Int16::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received steering command: %d", msg->data);
  }

  void motor_cb(const std_msgs::msg::Int16::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received motor forward command: %d", msg->data);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dt_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr steer_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr motor_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeUcBridge>());
  rclcpp::shutdown();
  return 0;
}
