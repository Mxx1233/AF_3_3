#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

// =======================
// Einfache Konfiguration
// =======================
#define WHEEL_CIR_FERENCE  0.22   // Radumfang in Metern (z.B. 0.22 = 22 cm)
#define DIRECTION_SIGN     1      // +1: Vorwärts, -1: falls Drehrichtung invertiert

// Topics
#define ODOM_TOPIC         "/odom"
#define IMU_TOPIC          "/imu_data"
#define DT8_TOPIC          "/dt8_data"   // Zeit für eine Radumdrehung
#define DT_TOPIC          "/dt_data"   // Zeit für eine 1/8*Radumdrehung

// Veröffentlichungsrate
#define PUBLISH_RATE_HZ    50.0   // Hz (z.B. 50 Hz)

class StateEstimationNode : public rclcpp::Node
{
public:
  StateEstimationNode()
  : rclcpp::Node("state_estimation_node"),
    v_lin(0.0),
    yaw(0.0),
    have_dt8(false),
    have_imu(false)
  {
    // Publisher: /odom
    odom_pub = create_publisher<nav_msgs::msg::Odometry>(ODOM_TOPIC, 10);

    // Subscriber: /dt_data
    dt_sub = create_subscription<std_msgs::msg::Float32>(
      DT_TOPIC, 10,
      std::bind(&StateEstimationNode::dtCallback, this, std::placeholders::_1));

    // Subscriber: /imu/data
    imu_sub = create_subscription<sensor_msgs::msg::Imu>(
      IMU_TOPIC, 10,
      std::bind(&StateEstimationNode::imuCallback, this, std::placeholders::_1));

    // Timer für periodische Integration & Publikation
    last_time = now();
    timer = create_wall_timer(
      std::chrono::duration<double>(1.0 / PUBLISH_RATE_HZ),
      std::bind(&StateEstimationNode::update, this));

    RCLCPP_INFO(get_logger(), "state_estimation_node gestartet (U=%.3f m, rate=%.1f Hz)",
                WHEEL_CIR_FERENCE, PUBLISH_RATE_HZ);
  }

private:
  // ======== Callbacks ========

  // Callback: dt8 (Sekunden pro Radumdrehung)
  void dtCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    last_dt_time = now();
    const float period = msg->data;
    if (period > 0.0f && std::isfinite(period)) {
      v_lin = (1.0 / 8.0) * DIRECTION_SIGN * (WHEEL_CIR_FERENCE / static_cast<double>(period));
      have_dt8 = true;
    } else {
      v_lin = 0.0;
    }
  }

  // Callback: IMU
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_imu = *msg;
    have_imu = true;
  }

  // ======== Hauptlogik ========
  void update()
  {
    rclcpp::Time t_now = now();
    double dt = (t_now - last_time).seconds();
    last_time = t_now;

    if (!have_dt8 || !have_imu || dt <= 0.0) {
      return; // warte bis beide Sensoren da sind
    }

    // ---- Integration der Gierrate w (rad/s) ----
    double w = last_imu.angular_velocity.z; // Drehgeschwindigkeit um Z-Achse
    yaw += w * dt;                          // einfache Integration
    normalizeYaw();                         // auf [-pi, pi]

    // ---- Quaternion aus Yaw berechnen ----
    geometry_msgs::msg::Quaternion q = yawToQuaternion(yaw);

    // ---- Odometry-Nachricht füllen ----
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = t_now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    // Orientierung (nur Yaw)
    odom.pose.pose.orientation = q;

    // Timeout für v_lin (falls dt8 ausbleibt)
    double time_since_dt8 = (t_now - last_dt_time).seconds();
    if (time_since_dt8 > 0.5) {
      v_lin = 0.0; // setze Geschwindigkeit auf 0
    }

    // Geschwindigkeit
    odom.twist.twist.linear.x = v_lin;
    odom.twist.twist.angular.z = w;


    // Position (2D) aus Integration v_lin und Yaw (Genauigkeit ist begrenzt)!
    odom.pose.pose.position.x += v_lin * std::cos(yaw) * dt;
    odom.pose.pose.position.y += v_lin * std::sin(yaw) * dt;
    odom.pose.pose.position.z = 0.0;  // flach auf Boden

    // ---- Publizieren ----
    odom_pub->publish(odom);
  }

  // ======== Hilfsfunktionen ========

  // Wandelt Yaw in Quaternion um (Rotation um Z)
  geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
  }

  void normalizeYaw()
  {
    if (yaw > M_PI) {yaw -= 2.0 * M_PI;}
    if (yaw < -M_PI) {yaw += 2.0 * M_PI;}
  }

  // ======== ROS Schnittstellen ========
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr dt_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::TimerBase::SharedPtr timer;

  // ======== Zustände ========
  rclcpp::Time last_time;
  rclcpp::Time last_dt_time;
  sensor_msgs::msg::Imu last_imu;
  double v_lin;
  double yaw;
  bool have_dt8;
  bool have_imu;
};

// ======== main ========
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimationNode>());
  rclcpp::shutdown();
  return 0;
}
