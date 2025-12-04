#ifndef LOW_LEVEL_PKG_UC_MAPPER_HPP
#define LOW_LEVEL_PKG_UC_MAPPER_HPP

#include <cmath>
#include <algorithm>

class UcMapper {
public:
  // Konfiguration der Struktur
  struct  Config
  {
    // Konfiguration der Lenkung
    double angle_steering_left_grad = -25.0;
    double angle_steering_right_grad = 25.0;
    double angle_left_rad = M_PI / 180.0 * angle_steering_left_grad;
    double angle_right_rad = M_PI / 180.0 * angle_steering_right_grad;

    int set_steering_left_max = -250;
    int set_steering_right_max = 250;

    int set_motor_level_forward_max = 1000;
    int set_motor_level_backward_max = 500;
  };

  struct CommandOutput
  {
    int16_t steering = 0;
    int16_t motor_fwd = 0;
    int16_t motor_bwd = 0;
  };

  explicit  UcMapper(const Config & cfg)
  : cfg_(cfg) {}

  CommandOutput mapCommand(double steering_angle_rad, float motor_level) const
  {
    CommandOutput output;

    // Lenkung umrechnen
    if (steering_angle_rad <= cfg_.angle_left_rad) {
      output.steering = cfg_.set_steering_left_max;
    } else if (steering_angle_rad >= cfg_.angle_right_rad) {
      output.steering = cfg_.set_steering_right_max;
    } else {
      // Linear interpolieren
      double range_x = cfg_.angle_right_rad - cfg_.angle_left_rad;
      double range_y = cfg_.set_steering_right_max - cfg_.set_steering_left_max;
      output.steering = static_cast<int16_t>(
        cfg_.set_steering_left_max +
        (steering_angle_rad - cfg_.angle_left_rad) * (range_y / range_x));
    }

    // Motorlevel umrechnen
    if (motor_level > 0.0f) {
      // Vorwärts fahren
      output.motor_fwd = static_cast<int16_t>(
        std::min(motor_level, 1.0f) * cfg_.set_motor_level_forward_max);
      output.motor_bwd = 0;
    } else if (motor_level < 0.0f) {
      // Rückwärts fahren
      output.motor_bwd = static_cast<int16_t>(
        std::min(-motor_level, 1.0f) * cfg_.set_motor_level_backward_max);
      output.motor_fwd = 0;
    } else {
      // Motor aus
      output.motor_fwd = 0;
      output.motor_bwd = 0;
    }

    return output;
  }

private:
  Config cfg_;
};
#endif  // LOW_LEVEL_PKG_UC_MAPPER_HPP
