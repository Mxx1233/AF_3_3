#ifndef WS_TEMPLATE_A_HPP
#define WS_TEMPLATE_A_HPP

#include "rclcpp/rclcpp.hpp"
#include "psaf_configuration/configuration.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp" 
#include "geometry_msgs/msg/point.hpp"
#include "opencv2/opencv.hpp"

#include "cv_bridge/cv_bridge.hpp"

#include "rusty_racer_interfaces/msg/point.hpp"
#include "rusty_racer_interfaces/msg/lane_marking.hpp"

#define LANE_DETECTION_OUTSIDE_NODE "lane_detection_outside"
#define LANE_MARKINGS_TOPIC "/lane_detection/lane_markings" 

using namespace cv;
using namespace std;

class LaneDetectionOutsideNode : public rclcpp::Node
{
public:
    LaneDetectionOutsideNode();

    // Homography Matrix (Perspective Transform)
    double homography_matrix[9] = {
        -6.2365349709456925,    -10.097157885480069,   4268.640024997635,
        -0.34872955386066273,   -43.127498169861965,   5909.25216167306,
        -0.00032736275435828107, -0.03083930850079371,    1.0
    };

protected:
    Mat output_img;
    std::vector<std::vector<cv::Point>> lane_marking_;

    void update();
    void processImageOutercircle(const Mat& img, int padding);

    std::pair<std::vector<cv::Point>, std::vector<cv::Rect>> sliding_window_sampling_right_line(
            const cv::Mat& image, cv::Point base_point, int window_width = 120, int window_height = 40);

    std::pair<std::vector<cv::Point>, std::vector<cv::Rect>> sliding_window_sampling_center_line(
            const cv::Mat& image, cv::Point base_point, int window_width = 120, int window_height = 15);

    // [NEW] Helper to draw lanes on the original street view
    void drawProjectedLane(cv::Mat& img, const std::vector<cv::Point>& lane_points, const cv::Mat& H_inv, const cv::Scalar& color);

private:
    cv::Mat current_image_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    
    // Corrected type from rusty_racer_interfaces
    rclcpp::Publisher<rusty_racer_interfaces::msg::LaneMarking>::SharedPtr lane_markings_publisher_;

    // [NEW] Debug Publisher
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_publisher_;

    // [NEW] Binarized Debug Publisher (Preserved from the updated file)
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr binarized_publisher_;

    void imageCallback(sensor_msgs::msg::Image::SharedPtr msg);
    void publishLaneMarkings(const std::vector<std::vector<cv::Point>>& lane_marking_);
};

#endif //WS_TEMPLATE_A_HPP