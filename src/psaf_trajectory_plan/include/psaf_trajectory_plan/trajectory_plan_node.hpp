#ifndef WS_TEMPLATE_TRAJECTORY_PLAN_NODE_HPP
#define WS_TEMPLATE_TRAJECTORY_PLAN_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "psaf_configuration/configuration.hpp"
#include "rusty_racer_interfaces/msg/trajectory.hpp"
#include "rusty_racer_interfaces/msg/lane_marking.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "std_msgs/msg/int16.hpp"

// [NEW] Include for the specific deviation message
#include "rusty_racer_interfaces/msg/lane_deviation.hpp"

using namespace cv;
using namespace std;

class TrajectoryPlanNode : public rclcpp::Node
{
public:
  TrajectoryPlanNode();

    // Homography Matrix (Bird's Eye View)
    // Updated with new calibration data
  double homography_matrix[9] = {
    -6.2365349709456925, -10.097157885480069, 4268.640024997635,
    -0.34872955386066273, -43.127498169861965, 5909.25216167306,
    -0.00032736275435828107, -0.03083930850079371, 1.0
  };

protected:
  vector<Point2f> prev_transformed_trajectory;
  vector<Point2f> transformed_trajectory;
  rusty_racer_interfaces::msg::Trajectory trajectory_msg;

  std::vector<cv::Point> calculate_trajectory(
    const std::vector<cv::Point> & right_lane,
    const std::vector<cv::Point> & center_lane,
    const std::vector<cv::Point> & left_lane);

  std::vector<cv::Point2f> transform_to_car_coordinate_system(
    const std::vector<cv::Point> & trajectory);
  double quadraticFunc(double y, double a, double b, double c);

  int findClosestPointIndex(const vector<Point2f> & trajectory, float target_x);

  cv::Mat polyFit(const std::vector<double> & x, const std::vector<double> & y, int order);

    // Helper to draw visual debug
  void drawProjectedTrajectory(
    cv::Mat & img, const std::vector<cv::Point> & trajectory_points,
    const cv::Mat & H_inv);

  bool if_initialized = true;
  int current_state_ = -1;

    // Method to calculate deviations
  void calculateAndPublishDeviation(const std::vector<cv::Point2f> & trajectory_mm);

private:
    // Standard subscribers
  rclcpp::Subscription<rusty_racer_interfaces::msg::LaneMarking>::SharedPtr lane_markings_subscriber_;
  rclcpp::Publisher<rusty_racer_interfaces::msg::Trajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr err_publisher_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr state_subscriber_;

    // Image / Debug handling
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_publisher_;

    //tuning values
  cv::Mat current_image_;
  std::vector<cv::Point2f> last_valid_trajectory_;
  int blind_frame_count_ = 0;
  const int MAX_BLIND_FRAMES = 15;
  float prev_heading_error_ = 0.0f;

    // Publisher for the deviation
  rclcpp::Publisher<rusty_racer_interfaces::msg::LaneDeviation>::SharedPtr deviation_publisher_;

  void lane_markingCallback(rusty_racer_interfaces::msg::LaneMarking::SharedPtr LaneMarking);
  void imageCallback(sensor_msgs::msg::Image::SharedPtr msg);
  void publishTrajectory(const vector<cv::Point2f> & transformed_trajectory);
  void stateCallback(std_msgs::msg::Int16::SharedPtr msg);
};

#endif //WS_TEMPLATE_TRAJECTORY_PLAN_NODE_HPP