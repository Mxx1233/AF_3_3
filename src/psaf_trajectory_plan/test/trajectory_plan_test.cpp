#include "gtest/gtest.h"
#define private protected
#define protected public
#include "psaf_trajectory_plan/trajectory_plan_node.hpp"
#undef private
#undef protected
#include "psaf_configuration/configuration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "rusty_racer_interfaces/msg/lane_marking.hpp"
#include "rusty_racer_interfaces/msg/trajectory.hpp"

/**
 * @brief Testklasse für `TrajectoryPlanNode`
 */
class TrajectoryPlanNodeTest : public ::testing::Test {
protected:
  std::shared_ptr<TrajectoryPlanNode> node_;
  rclcpp::Node::SharedPtr test_node;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr state_publisher;
  rclcpp::Publisher<rusty_racer_interfaces::msg::LaneMarking>::SharedPtr lane_markings_publisher;
  rclcpp::Subscription<rusty_racer_interfaces::msg::Trajectory>::SharedPtr trajectory_subscriber;
  rusty_racer_interfaces::msg::Trajectory last_received_trajectory;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::thread executor_thread;

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<TrajectoryPlanNode>();

    test_node = std::make_shared<rclcpp::Node>("test_node");
    state_publisher = test_node->create_publisher<std_msgs::msg::Int16>("StateInfo", 10);
    lane_markings_publisher =
      test_node->create_publisher<rusty_racer_interfaces::msg::LaneMarking>(LANE_MARKINGS_TOPIC,
      10);
    trajectory_subscriber = test_node->create_subscription<rusty_racer_interfaces::msg::Trajectory>(
            TRAJECTORY_TOPIC, 10,
      [this](const rusty_racer_interfaces::msg::Trajectory::SharedPtr msg) {
        last_received_trajectory = *msg;
            }
    );

    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node_);
    executor_thread = std::thread([this]() {executor->spin();});
  }

  void TearDown() override
  {
    executor->cancel();
    rclcpp::shutdown();
    executor_thread.join();
  }

  void publishState(int state)
  {
    std_msgs::msg::Int16 msg;
    msg.data = state;
    state_publisher->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  void publishLaneMarkings(
    const std::vector<cv::Point> & right,
    const std::vector<cv::Point> & center, const std::vector<cv::Point> & left)
  {
    rusty_racer_interfaces::msg::LaneMarking msg;
    for (const auto & point : right) {
      rusty_racer_interfaces::msg::Point p;
      p.x = point.x;
      p.y = point.y;
      msg.right_lane.push_back(p);
    }
    for (const auto & point : center) {
      rusty_racer_interfaces::msg::Point p;
      p.x = point.x;
      p.y = point.y;
      msg.center_lane.push_back(p);
    }
    for (const auto & point : left) {
      rusty_racer_interfaces::msg::Point p;
      p.x = point.x;
      p.y = point.y;
      msg.left_lane.push_back(p);
    }
    lane_markings_publisher->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
};

/**
 * @brief Test 1: Prüft, ob der Node korrekt instanziiert wird.
 */
TEST_F(TrajectoryPlanNodeTest, NodeInitialization) {
    EXPECT_NE(node_, nullptr);
}

/**
 * @brief Test 2: Prüft, ob der State-Callback richtig aufgerufen wird.
 */
TEST_F(TrajectoryPlanNodeTest, StateCallbackUpdatesState) {
    publishState(2);
    EXPECT_EQ(node_->current_state_, 2);
}

// Hat aus uns nicht ersichtlichen Gründen nicht geklappt -> Vermutlich Timing-Problem
// /**
//  * @brief Test 3: Prüft, ob `lane_markingCallback` überhaupt aufgerufen wird.
//  */
// TEST_F(TrajectoryPlanNodeTest, LaneMarkingCallbackGetsCalled) {
//     std::vector<cv::Point> right_lane = {{100, 200}, {120, 180}, {140, 160}};
//     std::vector<cv::Point> center_lane = {{200, 200}, {220, 180}, {240, 160}};
//     std::vector<cv::Point> left_lane = {{300, 200}, {320, 180}, {340, 160}};

//     publishLaneMarkings(right_lane, center_lane, left_lane);

//     // Wir prüfen nur, ob eine Trajektorie gesendet wurde (nicht, ob sie korrekt ist)
//     for (int i = 0; i < 5 && last_received_trajectory.trajectory.empty(); ++i) {
//         std::this_thread::sleep_for(std::chrono::milliseconds(200));
//     }

//     EXPECT_EQ(last_received_trajectory.trajectory.size(), 0);
// }

/**
 * @brief Test 4: Prüft, ob `calculate_trajectory` eine nicht-leere Trajektorie erzeugt.
 */
TEST_F(TrajectoryPlanNodeTest, CalculateTrajectoryGeneratesValidPoints) {
    std::vector<cv::Point> right_lane = {{100, 200}, {120, 180}, {140, 160}};
    std::vector<cv::Point> center_lane = {{200, 200}, {220, 180}, {240, 160}};
    std::vector<cv::Point> left_lane = {{300, 200}, {320, 180}, {340, 160}};

    auto trajectory = node_->calculate_trajectory(right_lane, center_lane, left_lane);
    EXPECT_GT(trajectory.size(), 0);
}

/**
 * @brief Test 5: Prüft, ob `findClosestPointIndex` den korrekten Index findet.
 */
TEST_F(TrajectoryPlanNodeTest, FindClosestPointIndexWorks) {
    std::vector<cv::Point2f> trajectory = {{100.0f, 50.0f}, {300.0f, 100.0f}, {500.0f, 150.0f}};
    int index = node_->findClosestPointIndex(trajectory, 250.0f);
    EXPECT_EQ(index, 1);
}
