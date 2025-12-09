#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
// TODO: Replace 'rusty_racer_interfaces' with your actual message package name
#include "rusty_racer_interfaces/msg/lane_deviation.hpp"

using namespace std::chrono_literals;

class FakeVisionNode : public rclcpp::Node
{
public:
  FakeVisionNode()
  : Node("fake_vision_node")
  {
    // Create a publisher on the topic "/lane_deviation"
    // Queue size is set to 10
    publisher_ =
      this->create_publisher<rusty_racer_interfaces::msg::LaneDeviation>("/lane_deviation", 10);

    // Create a timer to fire every 50ms (1000ms / 20Hz = 50ms)
    timer_ = this->create_wall_timer(
      10ms, std::bind(&FakeVisionNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Fake Vision Node started. Publishing at 20Hz.");
  }

private:
  int phase_counter = 0;
  double turning = 1.0;
  void timer_callback()
  {
    auto message = rusty_racer_interfaces::msg::LaneDeviation();
    phase_counter++;
    if (phase_counter > 100) {
      phase_counter = 0;
      turning = -1.0 * turning;
    }

    // 1. Fill the Header
    message.header.stamp = this->now();
    message.header.frame_id = "base_link";

    // 2. Fill the Data based on your requirements
    message.lateral_error = 0;  // Always -0.1
    message.curvature = 0.0f;

    // You didn't specify heading_error, so initializing to 0.0
    // (Or calculated based on geometry if needed)
    message.heading_error = turning * 20 / 360 * M_PI * 1.0f;

    // 3. Publish
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<rusty_racer_interfaces::msg::LaneDeviation>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeVisionNode>());
  rclcpp::shutdown();
  return 0;
}
