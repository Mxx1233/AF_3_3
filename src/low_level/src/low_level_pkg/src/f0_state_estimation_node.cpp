#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

// =======================
// Einfache Konfiguration
// =======================
// Radparameter

// #define WHEEL_RADIUS_M     0.10     // Radradius in Metern (z.B. 0.10 = 10 cm)
#define WHEEL_CIR_FERENCE  0.10     // Radumfang in Metern (z.B. 0.10 = 10 cm)
#define DIRECTION_SIGN     1        // +1: Vorwärts, -1: falls Drehrichtung invertiert

// Topics
#define ODOM_TOPIC         "/odom"
#define IMU_TOPIC          "/imu/data"
#define DT8_TOPIC          "/dt8_data"

// Frames
#define ODOM_FRAME_ID      "odom"
#define BASE_FRAME_ID      "base_link"

// Veröffentlichungsrate
#define PUBLISH_RATE_HZ    50.0     // Hz (z.B. 50 Hz)

// Kovarianzen (sehr grob, später projektspezifisch anpassen oder auf 0 lassen)
#define POSE_XY_VAR        0
#define POSE_YAW_VAR       0
#define TWIST_VX_VAR       0
#define TWIST_WZ_VAR       0

class F0StateEstimationNode : public rclcpp::Node {
public:
  F0StateEstimationNode()
  : rclcpp::Node("f0_state_estimation_node"),
    x_(0.0), y_(0.0), yaw_(0.0), v_lin_(0.0),
    have_imu_(false), have_dt8_(false)
  {
    // Publisher: wir senden Odometry auf /odom
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(ODOM_TOPIC, 10);

    // Subscriber: IMU (liefert Orientierung als Quaternion + Winkelgeschwindigkeiten)
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      IMU_TOPIC, 10, std::bind(&F0StateEstimationNode::imuCallback, this, std::placeholders::_1));

    // Subscriber: dt8 (Float32) = Sekunden pro Radumdrehung
    // -> daraus berechnen wir die Vorwärtsgeschwindigkeit v = Umfang / Periode
    dt8_sub_ = create_subscription<std_msgs::msg::Float32>(
      DT8_TOPIC, 10, std::bind(&F0StateEstimationNode::dt8Callback, this, std::placeholders::_1));

    // Timer ruft periodisch update() auf, um Pose zu integrieren und /odom zu publizieren
    last_time_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / PUBLISH_RATE_HZ),
      std::bind(&F0StateEstimationNode::update, this));

    RCLCPP_INFO(get_logger(), "f0_state_estimation_node gestartet (r=%.3f m, rate=%.1f Hz)",
                WHEEL_CIR_FERENCE, PUBLISH_RATE_HZ);
  }

private:
  // Hilfsfunktion: Yaw (Drehung um Z-Achse) aus Quaternion bestimmen
  // Formel für Z-Y-X (Yaw-Pitch-Roll): yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
  static double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  // IMU-Callback: speichert die letzte IMU-Nachricht und aktualisiert den Yaw-Winkel
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_imu_ = *msg;
    yaw_ = yawFromQuat(msg->orientation);
    have_imu_ = true;
  }

  // dt8-Callback: zeit pro Umdrehung [s] -> v [m/s] = Umfang / Periode
  void dt8Callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    const float period = msg->data;
    if (period > 0.0f && std::isfinite(period)) {
      //const double circumference = 2.0 * M_PI * WHEEL_RADIUS_M;
      v_lin_ = DIRECTION_SIGN * (WHEEL_CIR_FERENCE / static_cast<double>(period));
      have_dt8_ = true;
    } else {
      // Ungültige Periode -> wir nehmen 0 m/s an (sicheres Fallback)
      v_lin_ = 0.0;
    }
  }

  // Periodische Aktualisierung:
  // 1) Delta-Zeit bestimmen
  // 2) 2D-Pose integrieren (x,y mit v und Yaw)
  // 3) Odometry-Nachricht füllen und publizieren
  void update()
  {
    const rclcpp::Time t_now = now();
    const double dt = (t_now - last_time_).seconds();
    last_time_ = t_now;

    // Warten, bis mindestens eine IMU- und eine dt8-Nachricht eingetroffen sind
    if (!have_imu_ || !have_dt8_ || dt <= 0.0) {
      return;
    }

    // ------------------------------
    // 2D-Pose-Integration:
    // x,y ändern sich entlang der aktuellen Blick-/Fahrtrichtung (Yaw)
    // z bleibt 0 (Bodenfahrzeug)
    // ------------------------------
    x_ += v_lin_ * std::cos(yaw_) * dt;
    y_ += v_lin_ * std::sin(yaw_) * dt;

    // Odometry füllen
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = t_now;
    odom.header.frame_id = ODOM_FRAME_ID;     // Referenzrahmen (Welt / Odom)
    odom.child_frame_id = BASE_FRAME_ID;      // Roboter-selbst (am Chassis befestigt)

    // Pose: Position aus Integration, Orientierung direkt aus IMU
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = last_imu_.orientation;

    // Twist: Vorwärtsgeschwindigkeit (im Roboter-X), Yaw-Rate aus IMU
    odom.twist.twist.linear.x = v_lin_;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = last_imu_.angular_velocity.z;

    // Sehr einfache Kovarianzen (optional, hier nur wenige Felder sinnvoll > 0)
    // Pose-Cov: 6x6 (x y z roll pitch yaw)
    for (double & c : odom.pose.covariance) {c = 0.0;}
    odom.pose.covariance[0] = POSE_XY_VAR; // var(x)
    odom.pose.covariance[7] = POSE_XY_VAR; // var(y)
    odom.pose.covariance[35] = POSE_YAW_VAR; // var(yaw)

    // Twist-Cov: 6x6 (vx vy vz wx wy wz)
    for (double & c : odom.twist.covariance) {c = 0.0;}
    odom.twist.covariance[0] = TWIST_VX_VAR; // var(vx)
    odom.twist.covariance[35] = TWIST_WZ_VAR; // var(wz)

    // Publizieren
    odom_pub_->publish(odom);
  }

  // --- ROS I/O ---
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr dt8_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- Zustände ---
  rclcpp::Time last_time_;
  sensor_msgs::msg::Imu last_imu_{};
  double x_, y_, yaw_, v_lin_;
  bool have_imu_, have_dt8_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<F0StateEstimationNode>());
  rclcpp::shutdown();
  return 0;
}
