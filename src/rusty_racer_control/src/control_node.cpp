/**
 * @file control_node.cpp
 * @brief Rusty Racer Regelungsknoten
 * @author zx
 * @date 2025-12
 */

// ROS2 Header
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "std_msgs/msg/header.hpp"
#include <rusty_racer_interfaces/msg/lane_deviation.hpp>
#include <rusty_racer_interfaces/msg/motor_command.hpp>

// Standard-Bibliotheken
#include <cmath>
#include <memory>

// Regler-Komponenten (Logik in Header-Dateien)
#include "rusty_racer_control/common.h"
#include "rusty_racer_control/laengsfuehrung_controller.h"
#include "rusty_racer_control/motor_mapping.h"
#include "rusty_racer_control/lateral_controller.h"

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
    const double lat_kp = 0.5;
    const double lat_kd = 0.8;

    lateral_controller_ = std::make_unique<LateralController>(
            v_init, l, l_h, lat_kp, lat_kd
    );

        // Longitudinale Regelparameter (Geschwindigkeitsdomäne)
    pi_params_.Kp = 1.0;              // Proportionalverstärkung
    pi_params_.Ki = 0.01;              // Integralverstärkung
    pi_params_.v_min = 0.0;           // Minimale Geschwindigkeit [m/s]
    pi_params_.v_max = 2.9;           // Maximale Geschwindigkeit [m/s] (Messung erforderlich)

    pi_state_ = init_pi();

    v_ref_ = 0.8;      // Sollgeschwindigkeit 0.5 m/s

        // Initialisierung
    current_v_ = 0.0;
    current_psi_k_ = 0.0;
    last_update_time_ = this->now();

        // Subscriber erstellen
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&ControlNode::odomCallback, this, std::placeholders::_1)
    );

    lane_sub_ = this->create_subscription<rusty_racer_interfaces::msg::LaneDeviation>(
            "/lane_deviation", 10,
            std::bind(&ControlNode::laneCallback, this, std::placeholders::_1)
    );

        // Publisher erstellen
    motor_cmd_pub_ = this->create_publisher<rusty_racer_interfaces::msg::MotorCommand>(
            "/motor_command", 10
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
  void laneCallback(const rusty_racer_interfaces::msg::LaneDeviation::SharedPtr msg)
  {
        // Daten aus LaneDeviation-Nachricht extrahieren

    double y = msg->lateral_error;              // Querabweichung [m]
    double phi_k = msg->heading_error;          // Kursabweichung [rad]
        // double curvature = msg->curvature;   // Optional: Krümmung [1/m]

        // Abtastzeit berechnen
    auto current_time = this->now();
    double dt = (current_time - last_update_time_).seconds();
    if (dt <= 0.0 || dt > 0.1) {dt = 0.02;}

        // Longitudinalregelung: PI-Regler + Mapping
    double v_cmd = pi_step(pi_params_, pi_state_, v_ref_, current_v_, dt);
    // double v_cmd = 0.5;
    double motor_level = speed_to_motor_level(v_cmd, pi_params_.v_max);

        // Lateralregelung: PD-Regler
    double delta = lateral_controller_->compute(y, phi_k);

        // MotorCommand-Nachricht erstellen und veröffentlichen
    auto cmd = rusty_racer_interfaces::msg::MotorCommand();
    cmd.header = msg->header;                    // Header hier hinzugefügt
    cmd.motor_level = motor_level;              // Motorantriebsniveau [-1.0, 1.0]
    cmd.steering_angle = delta;                 // Lenkwinkel [rad]
    motor_cmd_pub_->publish(cmd);

        // Logging
        /*RCLCPP_INFO(this->get_logger(),
            "y=%+.3fm φ=%+.1f° | v_k=%.3f v_cmd=%.3f m_lv=%.3f | δ=%+.1f°",
            y,
            phi_k * 180.0 / M_PI,
            current_v_,
            v_cmd,
            motor_level,
            delta * 180.0 / M_PI
        );*/

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
  rclcpp::Subscription<rusty_racer_interfaces::msg::LaneDeviation>::SharedPtr lane_sub_;
  rclcpp::Publisher<rusty_racer_interfaces::msg::MotorCommand>::SharedPtr motor_cmd_pub_;

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
