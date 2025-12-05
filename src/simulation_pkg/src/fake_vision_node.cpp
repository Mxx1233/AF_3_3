#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" // Placeholder for LaneDetection

using namespace std::chrono_literals;
using LaneDetection = geometry_msgs::msg::Twist;

class FakeVisionNode : public rclcpp::Node
{
public:
  FakeVisionNode()
  : Node("fake_vision_node")
  {
    publisher_ = this->create_publisher<LaneDetection>("/perception/lane_detection", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&FakeVisionNode::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Fake Vision Node started. Publishing dummy lane data.");
  }

private:
  void timer_callback()
  {
    auto msg = LaneDetection();
        // Simulate a slight deviation to the right (y = -0.1m)
    msg.linear.x = -0.1;
        // Simulate a slight left curve in the path (theta_path = 5 degrees)
    msg.angular.z = 0.087;

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing fake lane detection: y=%.2f, theta=%.2f",
      msg.linear.x, msg.angular.z);
  }

  rclcpp::Publisher<LaneDetection>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeVisionNode>());
  rclcpp::shutdown();
  return 0;
}
