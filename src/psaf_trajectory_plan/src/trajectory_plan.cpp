//
// Created by psaf on 30.10.24.
//
#include "rclcpp/rclcpp.hpp"
#include "psaf_trajectory_plan/trajectory_plan_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<TrajectoryPlanNode> node = std::make_shared<TrajectoryPlanNode>();
    double update_frequency = node->declare_parameter("update_frequency", 125.0);
    rclcpp::WallRate rate(update_frequency);

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
}
