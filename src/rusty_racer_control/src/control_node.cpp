/**
 * @file control_node.cpp
 * @brief Rusty Racer Regelungsknoten
 * @author zx
 * @date 2025-12
 */

// ROS2 Header
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Standard-Bibliotheken
#include <cmath>
#include <memory>

// Regler-Komponenten (Logik in Header-Dateien)
#include "rusty_racer_control/common.h"
#include "rusty_racer_control/laengsfuehrung_controller.h"
#include "rusty_racer_control/motor_mapping.h"
#include "rusty_racer_control/lateral_controller.h"

// Temporäre Nachrichten-Definitionen
// TODO: Durch eigene MotorCommand-Nachricht ersetzen
using LaneDetection = geometry_msgs::msg::Twist;
using MotorCommand = geometry_msgs::msg::Twist;

/**
 * @class ControlNode
 * @brief Hauptregelungsknoten für Längs- und Querregelung
 */
class ControlNode : public rclcpp::Node
{
public:
  ControlNode()
  : Node("control_node")
  {
        // Fahrzeugparameter
    const double v_init = 0.5;        // Anfangsgeschwindigkeit [m/s]
    const double l = 0.257;           // Radstand [m]
    const double l_h = 0.0;           // Vision-Ausgabe: Hinterachsenabweichung

        // Laterale Regelparameter
    const double lat_kp = 1.5;
    const double lat_kd = 0.8;

    lateral_controller_ = std::make_unique<LateralController>(
            v_init, l, l_h, lat_kp, lat_kd
    );

        // Longitudinale Regelparameter (Geschwindigkeitsdomäne)
    pi_params_.Kp = 1.0;              // Proportionalverstärkung
    pi_params_.Ki = 1.5;              // Integralverstärkung
    pi_params_.v_min = 0.0;           // Minimale Geschwindigkeit [m/s]
    pi_params_.v_max = 1.5;           // Maximale Geschwindigkeit [m/s] (Messung erforderlich)

    pi_state_ = init_pi();

    v_ref_ = 0.5;      // Sollgeschwindigkeit 0.5 m/s

        // Initialisierung
    current_v_ = 0.0;
    current_psi_k_ = 0.0;
    last_update_time_ = this->now();

        // Subscriber erstellen
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&ControlNode::odomCallback, this, std::placeholders::_1)
    );

    lane_sub_ = this->create_subscription<LaneDetection>(
            "/perception/lane_detection", 10,
            std::bind(&ControlNode::laneCallback, this, std::placeholders::_1)
    );

        // Publisher erstellen
    motor_cmd_pub_ = this->create_publisher<MotorCommand>(
            "/control/motor_command", 10
    );

        // Startup-Logging
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Regelungsknoten gestartet");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Lateral: PD + Vorfilter");
    RCLCPP_INFO(this->get_logger(), "Longitudinal: PI-Geschwindigkeitsregelung + Mapping");
    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
    RCLCPP_INFO(this->get_logger(), "PI-Parameter:");
    RCLCPP_INFO(this->get_logger(), "  Kp = %.2f", pi_params_.Kp);
    RCLCPP_INFO(this->get_logger(), "  Ki = %.2f", pi_params_.Ki);
    RCLCPP_INFO(this->get_logger(), "  v_max = %.2f m/s", pi_params_.v_max);
    RCLCPP_INFO(this->get_logger(), "========================================");
  }

private:
    /**
     * @brief Odometrie-Callback
     */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_v_ = msg->twist.twist.linear.x;
    current_psi_k_ = extractYawFromQuaternion(msg->pose.pose.orientation);
    lateral_controller_->updateVelocity(current_v_);
    last_update_time_ = this->now();
  }

    /**
     * @brief Spurerkennungs-Callback - Hauptregelschleife
     */
  void laneCallback(const LaneDetection::SharedPtr msg)
  {
        // Daten extrahieren
    double y = msg->linear.x;               // Querabweichung [m]
    double theta_path = msg->angular.z;     // Pfadkurswinkel [rad]

        // Abtastzeit berechnen
    auto current_time = this->now();
    double dt = (current_time - last_update_time_).seconds();
    if (dt <= 0.0 || dt > 0.1) {dt = 0.02;}

        // Kursabweichung berechnen
    double phi_k = angleWrap(theta_path - current_psi_k_);

        // Longitudinalregelung: PI-Regler + Mapping
    double v_cmd = pi_step(pi_params_, pi_state_, v_ref_, current_v_, dt);
    double motor_level = speed_to_motor_level(v_cmd, pi_params_.v_max);

        // Lateralregelung: PD-Regler
    double delta = lateral_controller_->compute(y, phi_k);

        // Befehle veröffentlichen
    auto cmd = MotorCommand();
    cmd.linear.x = motor_level;         // Temporär: motor_level
    cmd.angular.z = delta;              // Temporär: steering_angle
    motor_cmd_pub_->publish(cmd);

        // Logging
    RCLCPP_INFO(this->get_logger(),
            "y=%+.3fm φ=%+.1f° | v_k=%.3f v_cmd=%.3f m_lv=%.3f | δ=%+.1f°",
            y,
            phi_k * 180.0 / M_PI,
            current_v_,
            v_cmd,
            motor_level,
            delta * 180.0 / M_PI
    );

    last_update_time_ = current_time;
  }

    /**
     * @brief Extrahiert Gierwinkel aus Quaternion
     */
  double extractYawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    tf2::Quaternion tf_quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 mat(tf_quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    return yaw;
  }

    // Membervariablen
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<LaneDetection>::SharedPtr lane_sub_;
  rclcpp::Publisher<MotorCommand>::SharedPtr motor_cmd_pub_;

  std::unique_ptr<LateralController> lateral_controller_;
  PIParams pi_params_;
  PIState pi_state_;

  double v_ref_;
  double current_v_;
  double current_psi_k_;
  rclcpp::Time last_update_time_;
};

/**
 * @brief Hauptfunktion
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
