#include "psaf_lane_detection_outside/lane_detection_outside_node.hpp"
#include <atomic>

std::atomic<int> file_counter(0);

LaneDetectionOutsideNode::LaneDetectionOutsideNode()
: Node(LANE_DETECTION_OUTSIDE_NODE)
{
  image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
    CAM_TOPIC_RGB, 10,
    std::bind(&LaneDetectionOutsideNode::imageCallback, this, std::placeholders::_1));

  // Keep the correct (working) message type from the “correct includes/messages” file
  lane_markings_publisher_ =
    this->create_publisher<rusty_racer_interfaces::msg::LaneMarking>(LANE_MARKINGS_TOPIC, 10);

  // Debug overlay publisher
  debug_publisher_ =
    this->create_publisher<sensor_msgs::msg::Image>("lane_detection/debug_overlay", 10);

  // [UPDATED FEATURE] Binarized image debug publisher (topic from updated code)
  // NOTE: This assumes your header defines `binarized_publisher_` similarly to debug_publisher_.
  binarized_publisher_ =
    this->create_publisher<sensor_msgs::msg::Image>("lane_detection/debug_binarized", 10);

  lane_marking_.resize(3);
}

// Call back function
void LaneDetectionOutsideNode::imageCallback(sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat image = cv_ptr->image;

  // -----------------------------------------------------------------------
  // [UPDATED] DIGITAL BLINDERS (ROI CROP)
  // -----------------------------------------------------------------------
  // Black out the top of the image so lane detection doesn't get confused by the horizon.
  // Start with 220. Increase if it still jumps lanes. Decrease if you can't see enough.
  int HORIZON_CROP = 250;
  if (HORIZON_CROP > 0 && HORIZON_CROP < image.rows) {
    cv::rectangle(
      image, cv::Point(0, 0), cv::Point(image.cols, HORIZON_CROP),
      cv::Scalar(0, 0, 0), cv::FILLED);
  }
  cv::rectangle(image, cv::Point(0, 0), cv::Point(355, image.rows),
              cv::Scalar(0, 0, 0), cv::FILLED);

  current_image_ = image;

  auto start = std::chrono::steady_clock::now();

  // [UPDATED] padding per updated code
  processImageOutercircle(current_image_, 10);

  auto end = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  // RCLCPP_INFO(this->get_logger(), "Processing time: %ld ms", duration);

  publishLaneMarkings(lane_marking_);
  lane_marking_.at(0).clear();
  lane_marking_.at(1).clear();
  lane_marking_.at(2).clear();
}

// Publish function
void LaneDetectionOutsideNode::publishLaneMarkings(const std::vector<std::vector<cv::Point>> & lane_marking_)
{
  rusty_racer_interfaces::msg::LaneMarking LaneMarking_msg;

  for (size_t i = 0; i < lane_marking_.size(); ++i) {
    const auto & lane = lane_marking_[i];
    for (const auto & point : lane) {
      rusty_racer_interfaces::msg::Point point_msg;
      point_msg.x = static_cast<int>(point.x);
      point_msg.y = static_cast<int>(point.y);

      if (i == 0) {
        LaneMarking_msg.left_lane.push_back(point_msg);
      } else if (i == 1) {
        LaneMarking_msg.center_lane.push_back(point_msg);
      } else if (i == 2) {
        LaneMarking_msg.right_lane.push_back(point_msg);
      }
    }
  }

  lane_markings_publisher_->publish(LaneMarking_msg);
  LaneMarking_msg.left_lane.clear();
  LaneMarking_msg.center_lane.clear();
  LaneMarking_msg.right_lane.clear();
}

// Helper Function: Projects Bird's Eye points back to Street View and draws them
void LaneDetectionOutsideNode::drawProjectedLane(
  cv::Mat & img, const std::vector<cv::Point> & lane_points,
  const cv::Mat & H_inv, const cv::Scalar & color)
{
  if (lane_points.empty()) {
    return;
  }

  // 1. Convert integer Points to Point2f for matrix multiplication
  std::vector<cv::Point2f> points_float;
  cv::Mat(lane_points).copyTo(points_float);

  // 2. Project points back to original image space (Inverse Perspective Mapping)
  std::vector<cv::Point2f> projected_points_float;
  cv::perspectiveTransform(points_float, projected_points_float, H_inv);

  // 3. Convert back to integer points for drawing
  std::vector<cv::Point> projected_points;
  projected_points.reserve(projected_points_float.size());
  for (const auto & p : projected_points_float) {
    // Filter out crazy points that might result from math errors at the horizon
    if (p.x > -5000 && p.x < 5000 && p.y > -5000 && p.y < 5000) {
      projected_points.push_back(p);
    }
  }

  // 4. Draw the lines
  const cv::Point * pts = (const cv::Point *) cv::Mat(projected_points).data;
  int npts = cv::Mat(projected_points).rows;

  cv::polylines(img, &pts, &npts, 1, false, color, 4, cv::LINE_AA);

  // Draw dots for specific points
  for (const auto & p : projected_points) {
    cv::circle(img, p, 5, color, -1);
  }
}

void LaneDetectionOutsideNode::processImageOutercircle(const Mat & img, int padding)
{
  // --- Image Processing Logic ---
  Mat bgrImg;
  if (img.channels() == 1) {
    cvtColor(img, bgrImg, COLOR_GRAY2BGR);
  } else {
    bgrImg = img;
  }

  // Keep the stronger horizon masking here as well (matches updated intent)
  cv::rectangle(
    bgrImg, cv::Point(0, 0), cv::Point(bgrImg.cols, 200),
    cv::Scalar(0, 0, 0), cv::FILLED);

  Mat gray;
  cvtColor(bgrImg, gray, COLOR_BGR2GRAY);

  Mat blurred;
  GaussianBlur(gray, blurred, Size(9, 9), 0);

  int blockSize = 55;
  int constSubtrahend = -55;

  // [UPDATED] Resize trick + publish binarized debug (from updated code)
  Mat small_blurred;
  resize(blurred, small_blurred, Size(), 0.25, 0.25);

  Mat small_binary;
  adaptiveThreshold(
    small_blurred, small_binary, 255,
    ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY,
    blockSize, constSubtrahend);

  Mat binary;
  resize(small_binary, binary, blurred.size());

  // [UPDATED] Morphological opening to remove small noise/white spots
  Mat opening_kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
  morphologyEx(binary, binary, MORPH_OPEN, opening_kernel, Point(-1, -1), 2);

  // [UPDATED FEATURE] Publish binarized image for debugging
  if (binarized_publisher_ && binarized_publisher_->get_subscription_count() > 0) {
    sensor_msgs::msg::Image::SharedPtr bin_msg;
    try {
      bin_msg = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        sensor_msgs::image_encodings::MONO8,
        binary).toImageMsg();
      binarized_publisher_->publish(*bin_msg);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception during binarized publish: %s", e.what());
    }
  }

  // Homography from header
  Mat homography(3, 3, CV_64F, homography_matrix);
  Mat binary_eagle;

  // warpPerspective to IPM view
  warpPerspective(binary, binary_eagle, homography, Size(640, 1088));

  Mat binary_eagle_padded;
  if (padding > 0) {
    copyMakeBorder(
      binary_eagle, binary_eagle_padded,
      padding, padding, padding, padding,
      BORDER_CONSTANT, Scalar(0));
  } else {
    binary_eagle_padded = binary_eagle;
  }

  Mat blurred_bird;
  GaussianBlur(binary_eagle_padded, blurred_bird, Size(5, 5), 0);
  Mat edges;
  Canny(blurred_bird, edges, 10, 150);

  Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
  Mat edges_dilated;
  dilate(edges, edges_dilated, kernel, Point(-1, -1), 1);

  kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
  Mat edges_closed;
  morphologyEx(edges_dilated, edges_closed, MORPH_CLOSE, kernel);

  vector<vector<Point>> contours;
  findContours(edges_closed, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
  cvtColor(binary_eagle_padded, output_img, COLOR_GRAY2BGR);

  // Height from calibration
  int img_h = 1088;

  // Search Region for the Right Lane
  vector<Point> fixed_rectangle_1 = {
    Point(320, img_h),
    Point(320, img_h - 300),
    Point(620, img_h - 300),
    Point(620, img_h)
  };
  Mat fixed_rect_mask_1 = Mat::zeros(binary_eagle_padded.size(), CV_8UC1);
  fillPoly(fixed_rect_mask_1, vector<vector<Point>>{fixed_rectangle_1}, Scalar(255));

  unordered_map<int, int> rightmost_x_by_row;
  vector<vector<Point>> filtered_contours;
  vector<pair<vector<Point>, double>> intersecting_contour_candidate;
  vector<Point> intersecting_contour;

  for (const auto & contour : contours) {
    double area = contourArea(contour);
    double perimeter = arcLength(contour, true);
    RotatedRect rect = minAreaRect(contour);
    double width = rect.size.width;
    double height = rect.size.height;
    double center_Point = rect.center.x;

    Mat mask = Mat::zeros(binary_eagle_padded.size(), CV_8UC1);
    drawContours(mask, vector<vector<Point>>{contour}, -1, Scalar(255), FILLED);

    Mat intersection_1;
    bitwise_and(fixed_rect_mask_1, mask, intersection_1);
    bool has_intersection_1 = countNonZero(intersection_1) > 0;

    if (has_intersection_1 && area > 1500 && max(height, width) > 150) {
      intersecting_contour_candidate.push_back(make_pair(contour, center_Point));
    }

    if (area > 500 && perimeter / area > 0.001) {
      double aspect_ratio = max(height, width) / min(height, width);
      if (min(width, height) > 5 && aspect_ratio > 3) {
        filtered_contours.push_back(contour);
      }
    }
  }

  if (!intersecting_contour_candidate.empty()) {
    auto min_element_it = min_element(
      intersecting_contour_candidate.begin(),
      intersecting_contour_candidate.end(),
      [](const pair<vector<Point>, double> & a, const pair<vector<Point>, double> & b) {
        return a.second < b.second;
      }
    );
    intersecting_contour = min_element_it->first;

    for (const auto & point : intersecting_contour) {
      int x = point.x;
      int y = point.y;
      if (rightmost_x_by_row.find(y) == rightmost_x_by_row.end()) {
        rightmost_x_by_row[y] = x;
      } else {
        rightmost_x_by_row[y] = min(rightmost_x_by_row[y], x);
      }
    }

    auto topmost_point = *min_element(
      intersecting_contour.begin(), intersecting_contour.end(),
      [](const Point & a, const Point & b) { return a.y < b.y; });

    int topmost_x = topmost_point.x;
    int topmost_y = topmost_point.y;

    for (int y = 0; y <= topmost_y; ++y) {
      if (rightmost_x_by_row.find(y) == rightmost_x_by_row.end()) {
        rightmost_x_by_row[y] = topmost_x;
      } else {
        rightmost_x_by_row[y] = min(rightmost_x_by_row[y], topmost_x);
      }
    }
  } else {
    intersecting_contour.clear();
  }

  if (!intersecting_contour.empty() &&
    find(filtered_contours.begin(), filtered_contours.end(), intersecting_contour) == filtered_contours.end())
  {
    filtered_contours.push_back(intersecting_contour);
  }

  vector<vector<Point>> final_contours;
  vector<Point> y_max_points;
  Mat binary_output_right = Mat::zeros(output_img.size(), CV_8UC1);
  Mat binary_output_center = Mat::zeros(output_img.size(), CV_8UC1);

  for (const auto & contour : filtered_contours) {
    bool keep_contour = true;

    if (!intersecting_contour.empty()) {
      for (const auto & point : contour) {
        int x = point.x;
        int y = point.y;
        if (rightmost_x_by_row.find(y) != rightmost_x_by_row.end() && x > rightmost_x_by_row[y]) {
          keep_contour = false;
          break;
        }
      }
    }

    if (keep_contour && contour != intersecting_contour) {
      drawContours(binary_output_center, vector<vector<Point>>{contour}, -1, Scalar(255), FILLED);
      final_contours.push_back(contour);

      auto max_point = *max_element(
        contour.begin(), contour.end(),
        [](const Point & a, const Point & b) { return a.y < b.y; });
      y_max_points.push_back(max_point);
    }
  }

  if (!intersecting_contour.empty()) {
    drawContours(binary_output_right, vector<vector<Point>>{intersecting_contour}, -1, Scalar(255), FILLED);
  }

  vector<cv::Point> center_lane;
  vector<cv::Point> right_lane;
  vector<cv::Point> left_lane;
  std::pair<std::vector<cv::Point>, std::vector<cv::Rect>> center_result;
  std::pair<std::vector<cv::Point>, std::vector<cv::Rect>> right_result;

  if (!intersecting_contour.empty()) {
    auto max_y_point = *max_element(
      intersecting_contour.begin(), intersecting_contour.end(),
      [](const Point & a, const Point & b) { return a.y < b.y; });

    Point right_lane_basepoint = max_y_point;

    // [UPDATED] Increased window width to better capture wider right lane
    right_result = sliding_window_sampling_right_line(binary_output_right, right_lane_basepoint, 90, 50);
    right_lane = right_result.first;

    if (!y_max_points.empty()) {
      Point center_lane_basepoint;
      std::vector<Point> y_max_points_sorted = y_max_points;
      std::sort(
        y_max_points_sorted.begin(), y_max_points_sorted.end(),
        [](const Point & a, const Point & b) { return a.y > b.y; });

      if (y_max_points_sorted.size() == 1) {
        center_lane_basepoint = y_max_points_sorted[0];
      } else {
        std::vector<Point> top_two_y_points = {y_max_points_sorted[0], y_max_points_sorted[1]};
        center_lane_basepoint = *std::max_element(
          top_two_y_points.begin(), top_two_y_points.end(),
          [](const Point & a, const Point & b) { return a.x < b.x; });
      }

      // [UPDATED] Reduced window width/height to prevent lane jumping
      center_result = sliding_window_sampling_center_line(binary_output_center, center_lane_basepoint, 60, 15);
      center_lane = center_result.first;
    } else {
      center_lane.clear();
    }

    left_lane.clear();
  } else {
    right_lane.clear();

    if (!y_max_points.empty()) {
      Point center_lane_basepoint;
      std::vector<Point> y_max_points_sorted = y_max_points;
      std::sort(
        y_max_points_sorted.begin(), y_max_points_sorted.end(),
        [](const Point & a, const Point & b) { return a.y > b.y; });

      if (y_max_points_sorted.size() == 1) {
        center_lane_basepoint = y_max_points_sorted[0];
      } else {
        std::vector<Point> top_two_y_points = {y_max_points_sorted[0], y_max_points_sorted[1]};
        center_lane_basepoint = *std::max_element(
          top_two_y_points.begin(), top_two_y_points.end(),
          [](const Point & a, const Point & b) { return a.x < b.x; });
      }

      center_result = sliding_window_sampling_center_line(binary_output_center, center_lane_basepoint, 60, 15);
      center_lane = center_result.first;
    } else {
      center_lane.clear();
    }

    left_lane.clear();
  }

  lane_marking_[0] = left_lane;
  lane_marking_[1] = center_lane;
  lane_marking_[2] = right_lane;

  // ==========================================================
  // VISUALIZATION SECTION
  // ==========================================================
  cv::Mat debug_view = bgrImg.clone();

  cv::Mat H(3, 3, CV_64F, homography_matrix);
  cv::Mat H_inv = H.inv();

  if (!left_lane.empty()) {
    drawProjectedLane(debug_view, left_lane, H_inv, cv::Scalar(0, 0, 255));
  }
  if (!center_lane.empty()) {
    drawProjectedLane(debug_view, center_lane, H_inv, cv::Scalar(0, 255, 0));
  }
  if (!right_lane.empty()) {
    drawProjectedLane(debug_view, right_lane, H_inv, cv::Scalar(255, 0, 0));
  }

  cv::putText(
    debug_view, "LANE PREDICTION (Street View)", cv::Point(20, 30),
    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);

  if (debug_publisher_ && debug_publisher_->get_subscription_count() > 0) {
    sensor_msgs::msg::Image::SharedPtr debug_msg;
    try {
      debug_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug_view).toImageMsg();
      debug_publisher_->publish(*debug_msg);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Could not convert debug image: %s", e.what());
    }
  }
}

// -----------------------------------------------------------------------------------------
// Existing sliding window functions (Unchanged)
// -----------------------------------------------------------------------------------------

std::pair<std::vector<cv::Point>, std::vector<cv::Rect>>
LaneDetectionOutsideNode::sliding_window_sampling_right_line(
  const cv::Mat & image, cv::Point base_point, int window_width, int window_height)
{
  std::vector<cv::Point> right_lane;
  cv::Point current_center = base_point;
  int img_height = image.rows;
  int img_width = image.cols;
  std::vector<cv::Rect> rectangles;

  while (true) {
    right_lane.push_back(current_center);
    int cx = current_center.x;
    int cy = current_center.y;

    int x_start = std::max(0, cx - window_width / 2);
    int x_end = std::min(img_width, cx + window_width / 2);
    int y_start = std::max(0, cy - window_height / 2);
    int y_end = std::min(img_height, y_start - window_height);

    if (y_end < 0 || y_start > image.rows || x_start < 0 || x_end > image.cols ||
      current_center.x < 0 || current_center.x > image.cols ||
      current_center.y < 0 || current_center.y > image.rows)
    {
      break;
    }

    int middle_row = (y_start + y_end) / 2;
    cv::Mat row_pixels = image.row(middle_row).colRange(x_start, x_end);
    bool found = false;

    for (int i = 0; i < row_pixels.cols; ++i) {
      if (row_pixels.at<uchar>(0, i) > 0) {
        current_center = cv::Point(x_start + i, middle_row);
        found = true;
        break;
      }
    }
    if (!found) {
      break;
    }

    x_start = std::max(0, current_center.x - window_width / 2);
    y_start = std::max(0, current_center.y - window_height / 2);

    if (y_end < 0 || y_start > image.rows || x_start < 0 || x_end > image.cols ||
      current_center.x < 0 || current_center.x > image.cols ||
      current_center.y < 0 || current_center.y > image.rows)
    {
      break;
    }

    rectangles.push_back(cv::Rect(x_start, y_start, window_width, window_height));
  }

  return std::make_pair(right_lane, rectangles);
}

std::pair<std::vector<cv::Point>, std::vector<cv::Rect>>
LaneDetectionOutsideNode::sliding_window_sampling_center_line(
  const cv::Mat & image, cv::Point base_point, int window_width, int window_height)
{
  std::vector<cv::Point> center_line_points;
  cv::Point current_center = base_point;
  int img_height = image.rows;
  int img_width = image.cols;
  std::vector<cv::Rect> rectangles;
  double distance;

  while (true) {
    center_line_points.push_back(current_center);
    int cx = current_center.x;
    int cy = current_center.y;

    int x_start = std::max(0, cx - window_width / 2);
    int x_end = std::min(img_width, cx + window_width / 2);
    int y_start = std::max(0, cy - window_height / 2);
    int y_end = std::min(img_height, y_start - window_height);

    if (y_end < 0 || y_start > image.rows || x_start < 0 || x_end > image.cols ||
      current_center.x < 0 || current_center.x > image.cols ||
      current_center.y < 0 || current_center.y > image.rows)
    {
      break;
    }

    int middle_row = (y_start + y_end) / 2;
    cv::Mat row_pixels = image.row(middle_row).colRange(x_start, x_end);

    bool found = false;

    for (int i = row_pixels.cols - 1; i >= 0; --i) {
      if (row_pixels.at<uchar>(0, i) > 0) {
        current_center = cv::Point(x_start + i, middle_row);
        found = true;
        break;
      }
    }

    if (!found) {
      int retry = 10;
      double dx = 0, dy = -1;

      if (center_line_points.size() >= 2) {
        cv::Point first_point = center_line_points[0];
        cv::Point last_point = center_line_points.back();

        dx = static_cast<double>(last_point.x - first_point.x);
        dy = static_cast<double>(last_point.y - first_point.y);
        double magnitude = std::max(1e-6, std::sqrt(dx * dx + dy * dy));
        dx /= magnitude;
        dy /= magnitude;

        if (dy == 0) {
          dy = -1e-6;
        }

        if (std::abs(-dx * window_height / dy) > 1) {
          distance = -dx * window_height / dy;
        } else {
          if (std::abs(-dx * window_height / dy) == 0) {
            distance = 1e-6;  // corrected assignment from == to =
          } else {
            distance = std::abs(-dx * window_height / dy) / (-dx * window_height / dy);
          }
        }
      }

      for (int attempt = 0; attempt < retry; ++attempt) {
        current_center.x += static_cast<int>(distance);

        x_start = std::max(0, current_center.x - window_width / 2);
        x_end = std::min(img_width, current_center.x + window_width / 2);
        y_start = std::max(0, current_center.y - window_height / 2);
        y_end = std::min(img_height, y_start - window_height);

        if (y_end < 0 || y_start > image.rows || x_start < 0 || x_end > image.cols ||
          current_center.x < 0 || current_center.x > image.cols ||
          current_center.y < 0 || current_center.y > image.rows)
        {
          break;
        }

        middle_row = (y_start + y_end) / 2;
        row_pixels = image.row(middle_row).colRange(x_start, x_end);
        current_center.x = (x_start + x_end) / 2;
        current_center.y = (y_start + y_end) / 2;

        if (y_end < 0 || y_start > image.rows || x_start < 0 || x_end > image.cols ||
          current_center.x < 0 || current_center.x > image.cols ||
          current_center.y < 0 || current_center.y > image.rows)
        {
          break;
        }

        for (int i = row_pixels.cols - 1; i >= 0; --i) {
          if (row_pixels.at<uchar>(0, i) > 0) {
            current_center = cv::Point(x_start + i, middle_row);
            found = true;
            break;
          }
        }

        if (found) {
          break;
        }
      }

      if (!found) {
        break;
      }
    }

    x_start = std::max(0, current_center.x - window_width / 2);
    y_start = std::max(0, current_center.y - window_height / 2);

    if (y_end < 0 || y_start > image.rows || x_start < 0 || x_end > image.cols ||
      current_center.x < 0 || current_center.x > image.cols ||
      current_center.y < 0 || current_center.y > image.rows)
    {
      break;
    }

    rectangles.push_back(cv::Rect(x_start, y_start, window_width, window_height));
  }

  return std::make_pair(center_line_points, rectangles);
}
