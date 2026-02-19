#include "psaf_trajectory_plan/trajectory_plan_node.hpp"
#include "psaf_configuration/configuration.hpp"
#include <atomic>
#include <cmath>
#include <algorithm>

std::atomic<int> file_counter(0);

TrajectoryPlanNode::TrajectoryPlanNode()
: Node(TRAJECTORY_PLAN_NODE)
{

  lane_markings_subscriber_ = this->create_subscription<rusty_racer_interfaces::msg::LaneMarking>(
            LANE_MARKINGS_TOPIC, 10,
            std::bind(&TrajectoryPlanNode::lane_markingCallback, this, std::placeholders::_1));

  image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            CAM_TOPIC_RGB, 10,
            std::bind(&TrajectoryPlanNode::imageCallback, this, std::placeholders::_1));

  state_subscriber_ = this->create_subscription<std_msgs::msg::Int16>(
            "StateInfo", 10,
            std::bind(&TrajectoryPlanNode::stateCallback, this, std::placeholders::_1));

  debug_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("trajectory/debug_overlay",
    10);

  trajectory_publisher_ = this->create_publisher<rusty_racer_interfaces::msg::Trajectory>(
            TRAJECTORY_TOPIC, 10);

  err_publisher_ = this->create_publisher<std_msgs::msg::Int16>("error", 10);

  deviation_publisher_ = this->create_publisher<rusty_racer_interfaces::msg::LaneDeviation>(
            "/lane_deviation", 10);

  RCLCPP_INFO(this->get_logger(), "Trajectory Node Started (S-Curve Agility Mode)");
}

void TrajectoryPlanNode::stateCallback(std_msgs::msg::Int16::SharedPtr msg)
{
  current_state_ = msg->data;
}

void TrajectoryPlanNode::imageCallback(sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    current_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void TrajectoryPlanNode::lane_markingCallback(
  rusty_racer_interfaces::msg::LaneMarking::SharedPtr LaneMarking)
{

  std::vector<cv::Point> right_lane;
  std::vector<cv::Point> center_lane;
  std::vector<cv::Point> left_lane;
  std::vector<cv::Point> trajectory;
  const float target_point = 1200.0f;

    // --- PARSING ---
  for (const auto & point : LaneMarking->right_lane) {
    right_lane.emplace_back(static_cast<int>(point.x), static_cast<int>(point.y));
  }
  for (const auto & point : LaneMarking->center_lane) {
    center_lane.emplace_back(static_cast<int>(point.x), static_cast<int>(point.y));
  }
  for (const auto & point : LaneMarking->left_lane) {
    left_lane.emplace_back(static_cast<int>(point.x), static_cast<int>(point.y));
  }

    // 1. Calculate Trajectory (Using the CUBIC SPLINE logic)
    // This allows the math to actually "bend" for the S-curve
  trajectory = calculate_trajectory(right_lane, center_lane, left_lane);

    // 2. Debug Visualization
  if (!current_image_.empty()) {
    cv::Mat debug_view = current_image_.clone();
    cv::Mat H(3, 3, CV_64F, homography_matrix);
    cv::Mat H_inv = H.inv();

        // Visual indicator if we are using Memory (Blind)
    if (blind_frame_count_ > 0) {
      cv::putText(debug_view, "MEMORY (BLIND)", cv::Point(20, 100),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 3);
    }

    drawProjectedTrajectory(debug_view, trajectory, H_inv);
    if (debug_publisher_->get_subscription_count() > 0) {
      try {
        auto debug_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8",
          debug_view).toImageMsg();
        debug_publisher_->publish(*debug_msg);
      } catch (cv_bridge::Exception & e) {
      }
    }
  }

    // 3. Transform to Car Coordinates
  transformed_trajectory = transform_to_car_coordinate_system(trajectory);

    // --- 4. MEMORY / PERSISTENCE LOGIC ---
    // This fixes the "Late Correction" caused by temporary camera blindness
    // when the car is pointed at the wall during the S-switch.
  const int MAX_BLIND_FRAMES = 15;

  if (transformed_trajectory.size() >= 3) {
        // We see the line -> Update memory
    last_valid_trajectory_ = transformed_trajectory;
    blind_frame_count_ = 0;
  } else {
        // We are blind -> Check if we can use memory
    if (blind_frame_count_ < MAX_BLIND_FRAMES && !last_valid_trajectory_.empty()) {
      transformed_trajectory = last_valid_trajectory_;
      blind_frame_count_++;
    } else {
            // Memory expired -> Truly lost
      transformed_trajectory.clear();
    }
  }

    // 5. Publish & Calculate
    // Only process if we have a valid trajectory (live or memory)
  if (!transformed_trajectory.empty()) {
    publishTrajectory(transformed_trajectory);
    calculateAndPublishDeviation(transformed_trajectory);
  }

    // Clean up local variable
  transformed_trajectory.clear();
}

// ---------------------------------------------------------------------------------------
// Deviation Logic [TUNED: 0.44m Lookahead, No Smoothing Lag]
// ---------------------------------------------------------------------------------------
void TrajectoryPlanNode::calculateAndPublishDeviation(
  const std::vector<cv::Point2f> & trajectory_mm)
{

  rusty_racer_interfaces::msg::LaneDeviation msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";

  if (trajectory_mm.size() < 3) {return;}

  // Sort by car_x (forward distance) and remove points that fold back.
  // The cubic spline in image space can produce points with non-monotone car_x
  // in tight curves — fitting y=f(x) on those gives garbage slopes.
  std::vector<cv::Point2f> sorted_traj = trajectory_mm;
  std::sort(sorted_traj.begin(), sorted_traj.end(),
    [](const cv::Point2f & a, const cv::Point2f & b) {return a.x < b.x;});

  // Keep only the monotone-forward portion: drop any point whose car_x is not
  // strictly greater than the previous accepted point (removes fold-back).
  std::vector<cv::Point2f> mono_traj;
  float last_x = -1e9f;
  for (const auto & p : sorted_traj) {
    if (p.x > last_x + 1.0f) {   // at least 1mm forward progress
      mono_traj.push_back(p);
      last_x = p.x;
    }
  }
  if (mono_traj.size() < 3) {return;}

  // --- RAW DIAGNOSTIC: log first, middle, last point of the monotone trajectory ---
  {
    const auto & front = mono_traj.front();
    const auto & mid = mono_traj[mono_traj.size() / 2];
    const auto & back = mono_traj.back();
    RCLCPP_INFO(this->get_logger(),
      "TRAJ pts=%zu  near(x=%.0f y=%.0f)  mid(x=%.0f y=%.0f)  far(x=%.0f y=%.0f)",
      mono_traj.size(),
      front.x, front.y,
      mid.x, mid.y,
      back.x, back.y);
  }

  // ═══════════════════════════════════════════════════════════════
  // TUNING PARAMETERS  ← adjust these to fix over/understeer
  // ═══════════════════════════════════════════════════════════════
  //
  // HEADING_LOOKAHEAD: where on the fitted curve the slope is read [m]
  //   Lower  (e.g. 0.15) → reacts to what is immediately ahead → less overshoot
  //   Higher (e.g. 0.50) → reacts far ahead → more anticipation but more overshoot
  const double HEADING_LOOKAHEAD = 0.15;

  // HEADING_SCALE: multiplier on heading_error [0..1]
  //   Lower  (e.g. 0.5) → reduces heading contribution → less overshoot in curves
  //   Higher (e.g. 1.0) → full heading correction
  const double HEADING_SCALE = 0.3;

  // CURVATURE_SCALE: multiplier on the curvature feedforward [0..1]
  //   Lower  (e.g. 0.5) → weaker feedforward, relies more on feedback
  //   Higher (e.g. 1.0) → full feedforward
  const double CURVATURE_SCALE = 0.7;

  const double SHORT_HORIZON_LIMIT = 0.60;
  const double LONG_HORIZON_LIMIT = 1.3;
  std::vector<double> x_long, y_long;
  std::vector<double> x_short, y_short;

  for (const auto & p : mono_traj) {
    double xm = p.x / 1000.0;
    double ym = p.y / 1000.0;

    if (xm > 0.05 && xm < LONG_HORIZON_LIMIT) {
      x_long.push_back(xm);
      y_long.push_back(ym);

      if (xm < SHORT_HORIZON_LIMIT) {
        x_short.push_back(xm);
        y_short.push_back(ym);
      }
    }
  }

  if (x_long.size() < 3) {return;}

  // --- 1. GLOBAL FIT (Quadratic) over full range for curvature ---
  cv::Mat C_long = polyFit(x_long, y_long, 2);
  double c0_long = C_long.at<double>(0, 0);
  double c1_long = C_long.at<double>(1, 0);
  double c2_long = C_long.at<double>(2, 0);

  // Curvature: evaluate slope at HEADING_LOOKAHEAD
  double slope_at_lookahead = c1_long + 2.0 * c2_long * HEADING_LOOKAHEAD;
  double curvature = (2.0 * c2_long) / std::pow(1.0 + slope_at_lookahead * slope_at_lookahead, 1.5);

  // --- 2. LOCAL FIT for lateral error and heading ---
  // Slope is read at HEADING_LOOKAHEAD — reduce this to reduce overshoot.
  // Lateral error is read at the same point so it matches the heading horizon.
  double lat_err = 0.0;
  double slope_at_car = 0.0;

  if (x_short.size() >= 3) {
    cv::Mat C_short = polyFit(x_short, y_short, 1);
    double c0_short = C_short.at<double>(0, 0);
    double c1_short = C_short.at<double>(1, 0);
    lat_err = c0_short + c1_short * HEADING_LOOKAHEAD;
    slope_at_car = c1_short;
  } else {
    // Fallback: quadratic evaluated at HEADING_LOOKAHEAD (inside fitted range)
    lat_err = c0_long + c1_long * HEADING_LOOKAHEAD + c2_long * HEADING_LOOKAHEAD *
      HEADING_LOOKAHEAD;
    slope_at_car = c1_long + 2.0 * c2_long * HEADING_LOOKAHEAD;
  }

  // Sign analysis (confirmed by TRAJ diagnostic logs):
  //   Left curve  → far car_y MORE NEGATIVE → slope NEGATIVE → curvature (2*c2) NEGATIVE
  //
  // Controller lateral_controller.h formula:
  //   φ*L = -kp*(y - y_target) - (kp+kd)*phi_k + κ*l
  //   phi_k > 0 → controller steers RIGHT (corrects leftward nose overshoot)
  //   phi_k < 0 → controller steers LEFT  (nose not yet pointing into left curve)
  //   κ     > 0 → feedforward steers LEFT  (left curve)
  //
  // Therefore:
  //   heading_error: slope<0 in left curve → pass raw (negative) → controller steers left ✓
  //   curvature:     κ<0 raw in left curve → negate → positive → feedforward steers left ✓
  //   lateral_error: sign passes through as-is
  msg.lateral_error = static_cast<float>(lat_err);

  // --- 3. HEADING ---
  double heading_raw = std::atan(slope_at_car);   // negative in left curve
  const double MAX_HEADING_ERROR = 0.7;
  heading_raw = std::clamp(heading_raw, -MAX_HEADING_ERROR, MAX_HEADING_ERROR);
  msg.heading_error = static_cast<float>(HEADING_SCALE * heading_raw);   // scale to reduce overshoot

  msg.curvature = static_cast<float>(CURVATURE_SCALE * -curvature);      // scale feedforward
  RCLCPP_INFO(this->get_logger(),
    "Curv: %.3f | Lat: %.3f | Head: %.3f | slope: %.3f | lookahead: %.2fm",
    msg.curvature, msg.lateral_error, msg.heading_error, slope_at_car, HEADING_LOOKAHEAD);

  deviation_publisher_->publish(msg);
}
// ---------------------------------------------------------------------------------------
// Helper: Polynomial Regression
// ---------------------------------------------------------------------------------------
cv::Mat TrajectoryPlanNode::polyFit(
  const std::vector<double> & x, const std::vector<double> & y,
  int order)
{
  cv::Mat X(x.size(), order + 1, CV_64F);
  cv::Mat Y(y.size(), 1, CV_64F);
  for (size_t i = 0; i < x.size(); ++i) {
    for (int j = 0; j <= order; ++j) {
      X.at<double>(i, j) = std::pow(x[i], j);
    }
    Y.at<double>(i, 0) = y[i];
  }
  cv::Mat C;
  cv::solve(X, Y, C, cv::DECOMP_QR);
  return C;
}

void TrajectoryPlanNode::publishTrajectory(const vector<cv::Point2f> & transformed_trajectory)
{
  trajectory_msg.trajectory.clear();
  for (auto it = transformed_trajectory.rbegin(); it != transformed_trajectory.rend(); ++it) {
    rusty_racer_interfaces::msg::FloatPoint point_msg;
    point_msg.x = static_cast<float>(it->x);
    point_msg.y = static_cast<float>(it->y);
    trajectory_msg.trajectory.push_back(point_msg);
  }
  trajectory_publisher_->publish(trajectory_msg);
}

void TrajectoryPlanNode::drawProjectedTrajectory(
  cv::Mat & img,
  const std::vector<cv::Point> & trajectory_points, const cv::Mat & H_inv)
{
  if (trajectory_points.empty()) {return;}
  std::vector<cv::Point2f> points_float;
  cv::Mat(trajectory_points).copyTo(points_float);
  std::vector<cv::Point2f> projected_points_float;
  cv::perspectiveTransform(points_float, projected_points_float, H_inv);

  std::vector<cv::Point> projected_points;
  for (const auto & p : projected_points_float) {
    if (p.x > -5000 && p.x < 5000 && p.y > -5000 && p.y < 5000) {
      projected_points.push_back(p);
    }
  }

  if (!projected_points.empty()) {
    const cv::Point * pts = (const cv::Point *) cv::Mat(projected_points).data;
    int npts = cv::Mat(projected_points).rows;
    cv::polylines(img, &pts, &npts, 1, false, cv::Scalar(0, 0, 255), 4, cv::LINE_AA);
    cv::putText(img, "Traj (Cubic/Agile)", cv::Point(20, 50),
            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
  }
}

// ---------------------------------------------------------------------------------------
// Helper: Transform to Car Coords
// ---------------------------------------------------------------------------------------
vector<Point2f> TrajectoryPlanNode::transform_to_car_coordinate_system(
  const vector<Point> & trajectory)
{
  vector<Point2f> transformed_trajectory;

    // Values from your calibration file
  const float etaEgo0_px = 1280.0f;
  const float xiEgo0_px = 320.0f;
  const float ego_scale = 0.64f;
  const int padding = 0;

  // [FIX] Mirroring Issue:
  // We re-introduce lateral_sign and use the inverted logic: (img_u - xiEgo0_px)
  const float lateral_sign = 1.0f;

  for (const auto & point : trajectory) {
    float img_u = static_cast<float>(point.x - padding);
    float img_v = static_cast<float>(point.y - padding);

    float car_x = (etaEgo0_px - img_v) / ego_scale;
    // New Logic: lateral_sign * (img_u - xiEgo0_px)
    float car_y = lateral_sign * (img_u - xiEgo0_px) / ego_scale;

    transformed_trajectory.emplace_back(car_x, car_y);
  }
  return transformed_trajectory;
}

// ---------------------------------------------------------------------------------------
// Helper: Calculate Trajectory (Cubic Spline)
// ---------------------------------------------------------------------------------------
vector<Point> TrajectoryPlanNode::calculate_trajectory(
  const vector<Point> & right_lane,
  const vector<Point> & center_lane, const vector<Point> & left_lane)
{

  // Updated to 160.0 for new scale (250mm * 0.64 = 160px)
  const double LANE_OFFSET_PX = 160.0;
  const int IMG_WIDTH = 640;
  const int PADDING = 20;

  vector<Point> trajectory;

    // --- 1. Generate Raw Offset Points ---
  if (!right_lane.empty()) {
    for (size_t i = 0; i < right_lane.size() - 1; ++i) {
      double dx = right_lane[i + 1].x - right_lane[i].x;
      double dy = right_lane[i + 1].y - right_lane[i].y;
      double len = sqrt(dx * dx + dy * dy);
      if(len == 0) {continue;}
      double nx = -dy / len; double ny = dx / len;
      double mx = (right_lane[i].x + right_lane[i + 1].x) / 2.0;
      double my = (right_lane[i].y + right_lane[i + 1].y) / 2.0;
      trajectory.emplace_back(cvRound(mx - nx * LANE_OFFSET_PX), cvRound(my - ny * LANE_OFFSET_PX));
    }
  } else if (!center_lane.empty()) {
    for (size_t i = 0; i < center_lane.size() - 1; ++i) {
      double dx = center_lane[i + 1].x - center_lane[i].x;
      double dy = center_lane[i + 1].y - center_lane[i].y;
      double len = sqrt(dx * dx + dy * dy);
      if(len == 0) {continue;}
      double nx = -dy / len; double ny = dx / len;
      double mx = (center_lane[i].x + center_lane[i + 1].x) / 2.0;
      double my = (center_lane[i].y + center_lane[i + 1].y) / 2.0;
      trajectory.emplace_back(cvRound(mx + nx * LANE_OFFSET_PX), cvRound(my + ny * LANE_OFFSET_PX));
    }
  }

    // Sort by Y (bottom of image to top)
  sort(trajectory.begin(), trajectory.end(), [](const Point & a, const Point & b) {
      return a.y > b.y;
                                                                                                    });
  vector<Point> final_trajectory;

    // --- 2. CUBIC SPLINE FITTING (Order 3) ---
    // Minimum 4 points needed for a cubic fit
  if (trajectory.size() > 4) {
    vector<double> y_points, x_points;
    for (const auto & point : trajectory) {
      y_points.push_back(point.y);
      x_points.push_back(point.x);
    }

        // Matrix A is now N x 4 (y^3, y^2, y, 1)
    Mat A(y_points.size(), 4, CV_64F);
    Mat B(x_points.size(), 1, CV_64F);

    for (size_t i = 0; i < y_points.size(); ++i) {
      double y = y_points[i];
      A.at<double>(i, 0) = y * y * y;       // Cubic term
      A.at<double>(i, 1) = y * y;           // Quadratic term
      A.at<double>(i, 2) = y;               // Linear term
      A.at<double>(i, 3) = 1.0;             // Intercept
      B.at<double>(i, 0) = x_points[i];
    }

    Mat params;
        // Use SVD for better stability with higher order polynomials
    solve(A, B, params, DECOMP_SVD);

    double a = params.at<double>(0, 0);
    double b = params.at<double>(1, 0);
    double c = params.at<double>(2, 0);
    double d = params.at<double>(3, 0);

    double y_min = *min_element(y_points.begin(), y_points.end());
    double y_max = *max_element(y_points.begin(), y_points.end());

        // Increase point density slightly for smoother control
    int num_points = static_cast<int>(trajectory.size()) * 20;
    double y_step = (y_max - y_min) / std::max(1, num_points - 1);

    for (int i = 0; i < num_points; ++i) {
      double gen_y = y_min + i * y_step;
            // x = ay^3 + by^2 + cy + d
      double gen_x = a * gen_y * gen_y * gen_y + b * gen_y * gen_y + c * gen_y + d;

      int xi = cvRound(gen_x);
      int yi = cvRound(gen_y);

      if (xi >= -PADDING && xi < (IMG_WIDTH + PADDING)) {
        if (i % 5 == 0) {final_trajectory.emplace_back(xi, yi);}
      }
    }
  } else {
        // Fallback if not enough points for Cubic fit
    final_trajectory = trajectory;
  }

  return final_trajectory;
}

double TrajectoryPlanNode::quadraticFunc(double y, double a, double b, double c)
{
  return a * y * y + b * y + c;
}

int TrajectoryPlanNode::findClosestPointIndex(const vector<Point2f> & trajectory, float target_x)
{
  int closest_index = -1;
  float min_distance = numeric_limits<float>::max();
  for (size_t i = 0; i < trajectory.size(); ++i) {
    float distance = abs(trajectory[i].x - target_x);
    if (distance < min_distance) {
      min_distance = distance;
      closest_index = i;
    }
  }
  return closest_index;
}