/**
 * @file firststeps.cpp
 * @brief the main method for the firststeps node. This function gets called by the launch file
 * @author PSAF
 * @date 2022-06-01
 */
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "psaf_firststeps/firststeps_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<FirstStepsNode> node = std::make_shared<FirstStepsNode>();
  rclcpp::WallRate rate(node->declare_parameter("update_frequency", rclcpp::PARAMETER_DOUBLE)
    .get<rclcpp::PARAMETER_DOUBLE>());

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->update();
    rate.sleep();
  }

  rclcpp::shutdown();
}
