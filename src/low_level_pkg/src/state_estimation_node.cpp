#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "low_level_pkg/state_estimation_core.hpp"

// =======================
// Einfache Konfiguration
// =======================

// Topics
#define ODOM_TOPIC         "/odom"
#define IMU_TOPIC          "/imu_data"
#define DT8_TOPIC          "/dt8_data"   // Zeit für eine Radumdrehung
#define DT_TOPIC          "/dt_data"   // Zeit für eine 1/8*Radumdrehung

// Veröffentlichungsrate
#define PUBLISH_RATE_HZ    100.0   // Hz (z.B. 50 Hz)

class StateEstimationNode : public rclcpp::Node
{
public:
  StateEstimationNode()
  : rclcpp::Node("state_estimation_node")
  {
    // Konfiguration der Logik von Core
    StateEstimatorCore::Config cfg;

    cfg.wheel_circumference = 0.22;
    cfg.direction_sign = 1;
    cfg.dt_timeout_sec = 0.5;

    estimator_ = std::make_unique<StateEstimatorCore>(cfg);

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
    timer = create_wall_timer(
      std::chrono::duration<double>(1.0 / PUBLISH_RATE_HZ),
      std::bind(&StateEstimationNode::update, this));

    RCLCPP_INFO(get_logger(), "state_estimation_node gestartet (U=%.3f m, rate=%.1f Hz)",
                cfg.wheel_circumference, PUBLISH_RATE_HZ);
  }

private:
  // ======== Callbacks ========

  // Callback: dt8 (Sekunden pro 1/8 Radumdrehung)
  void dtCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    estimator_->setDt8(msg->data, now().seconds());
  }

  // Callback: IMU
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    estimator_->setImu(msg->angular_velocity.z);
  }

  // ======== Hauptlogik ========
  void update()
  {
    rclcpp::Time t_now = now();

    if (!estimator_->update(t_now.seconds())) {
      return;
    }

    auto state = estimator_->getState();

    // ---- Quaternion aus Yaw berechnen ----
    geometry_msgs::msg::Quaternion q = yawToQuaternion(state.yaw);

    // ---- Odometry-Nachricht füllen ----
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = t_now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    // Orientierung (nur Yaw)
    odom.pose.pose.orientation = q;

    // Geschwindigkeit
    odom.twist.twist.linear.x = state.v_lin;
    odom.twist.twist.angular.z = state.w_z;


    // Position (2D) aus Integration v_lin und Yaw (Genauigkeit ist begrenzt)!
    // Für Rückwärtsfahrt noch nicht getestet
    odom.pose.pose.position.x = state.x;
    odom.pose.pose.position.y = state.y;
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

  std::unique_ptr<StateEstimatorCore> estimator_;

  // ======== ROS Schnittstellen ========
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr dt_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::TimerBase::SharedPtr timer;
};

// ======== main ========
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimationNode>());
  rclcpp::shutdown();
  return 0;
}
