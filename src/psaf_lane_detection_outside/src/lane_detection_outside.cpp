#include "rclcpp/rclcpp.hpp"
#include "psaf_lane_detection_outside/lane_detection_outside_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<LaneDetectionOutsideNode> node = std::make_shared<LaneDetectionOutsideNode>();
  double update_frequency = node->declare_parameter("update_frequency", 100.0);
  rclcpp::WallRate rate(update_frequency);

  while (rclcpp::ok()) {
        //auto start = std::chrono::steady_clock::now();
    rclcpp::spin_some(node);
        //node->update();
    rate.sleep();
        //rclcpp::shutdown();
        //auto end = std::chrono::steady_clock::now();
        //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        //RCLCPP_INFO(node->get_logger(), "===================================================== Lane_detection process time: %ld ms", duration);
  }

//    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//    rclcpp::spin_some(node);
//    rclcpp::shutdown();
}
