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

// Veröffentlichungsrate
#define PUBLISH_RATE_HZ    50.0     // Hz (z.B. 50 Hz)

class A1StateEstimationNode : public rclcpp::Node
{
public:
  A1StateEstimationNode()
  : rclcpp::Node("a1_state_estimation_node"),
    v_lin(0.0),
    have_dt8(false)
  {
        // Publisher: wir senden Odometry auf /odom
    odom_pub = create_publisher<nav_msgs::msg::Odometry>(ODOM_TOPIC, 10);

        // Subscriber: dt8 (Float32) = Sekunden pro Radumdrehung
        // -> daraus berechnen wir die Vorwärtsgeschwindigkeit v = Umfang / Periode
    dt8_sub = create_subscription<std_msgs::msg::Float32>(
            DT8_TOPIC, 10,
      std::bind(&A1StateEstimationNode::dt8Callback, this, std::placeholders::_1));

    last_time = now();
    timer = create_wall_timer(
            std::chrono::duration<double>(1.0 / PUBLISH_RATE_HZ),
            std::bind(&A1StateEstimationNode::update, this));

    RCLCPP_INFO(get_logger(), "a1_state_estimation_node gestartet (r=%.3f m, rate=%.1f Hz)",
                    WHEEL_CIR_FERENCE, PUBLISH_RATE_HZ);
  }

private:
    // dt8-Callback: zeit pro Umdrehung [s] -> v [m/s] = Umfang / Periode
  void dt8Callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    float period = msg->data;
    if (period > 0.0f && std::isfinite(period)) {
      v_lin = DIRECTION_SIGN * (WHEEL_CIR_FERENCE / static_cast<double>(period));
      have_dt8 = true;
    } else {
      v_lin = 0.0;
    }

  }
    // Periodische Aktualisierung:
    // 1) Delta-Zeit bestimmen
    // 2) Odometry-Nachricht füllen und publizieren
  void update()
  {
    rclcpp::Time t_now = now();
    double dt = (t_now - last_time).seconds();
    last_time = t_now;
    if (!have_dt8 || dt <= 0.0) {
      return;
    }
        // Odometry füllen
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = t_now;
        // odom.header.frame_id = ODOM_FRAME_ID;     // Referenzrahmen (
        // odom.child_frame_id = BASE_FRAME_ID;      // Roboter-selbst (am Chassis befestigt)

        // Twist: Vorwärtsgeschwindigkeit (im Roboter-X)
    odom.twist.twist.linear.x = v_lin;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
        // Publizieren
    odom_pub->publish(odom);
  }
    // --- ROS I/O ---
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr dt8_sub;
  rclcpp::TimerBase::SharedPtr timer;

    // --- Interne Zustandsvariablen ---
  rclcpp::Time last_time;
  double v_lin;
  bool have_dt8;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<A1StateEstimationNode>());
  rclcpp::shutdown();
  return 0;
}
