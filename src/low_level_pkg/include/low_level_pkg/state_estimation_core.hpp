//
// Created by ubuntu on 02.12.25.
//

#ifndef WISE_2025_26_GRUPPE_C_STATE_ESTIMATION_CORE_HPP
#define WISE_2025_26_GRUPPE_C_STATE_ESTIMATION_CORE_HPP

#include <algorithm>
#include <cmath>

class StateEstimatorCore {
public:
  // Konfiguration der Struktur
  struct Config
  {
    double wheel_circumference = 0.22;
    int direction_sign = 1;
    double dt_timeout_sec = 0.5;
  };

  // Zustände der Struktur
  struct State
  {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double v_lin = 0.0;
    double w_z = 0.0;
  };

  explicit StateEstimatorCore(const Config & cfg)
  : cfg_(cfg) {}

  void setDt8(double period_sec, double current_time_sec)
  {
    if (period_sec > 0.0f && std::isfinite(period_sec)) {
      state_.v_lin = (1.0 / 8.0) * cfg_.direction_sign *
        (cfg_.wheel_circumference / static_cast<double>(period_sec));
      last_dt_time_ = current_time_sec;
      have_dt8_ = true;
    } else {
      state_.v_lin = 0.0;
    }
  }

  void setImu(double angular_velocity_z)
  {
    state_.w_z = angular_velocity_z;
    have_imu_ = true;
  }

  bool update(double current_time_sec)
  {
    if (last_update_time_ < 0.0) {
      last_update_time_ = current_time_sec;
      return false;
    }

    double dt = current_time_sec - last_update_time_;
    last_update_time_ = current_time_sec;

    if (!have_dt8_ || !have_imu_ || dt <= 0.0) {
      return false;
    }

    if ((current_time_sec - last_dt_time_) > cfg_.dt_timeout_sec) {
      state_.v_lin = 0.0;
    }

    state_.yaw += state_.w_z * dt;
    normalizeYaw();

    state_.x += state_.v_lin * std::cos(state_.yaw) * dt;
    state_.y += state_.v_lin * std::sin(state_.yaw) * dt;

    return true;
  }

  State getState() const {return state_;}

private:
  void normalizeYaw()
  {
    if (state_.yaw > M_PI) {state_.yaw -= 2.0 * M_PI;}
    if (state_.yaw < -M_PI) {state_.yaw += 2.0 * M_PI;}
  }

  Config cfg_;
  State state_;

  bool have_dt8_ = false;
  bool have_imu_ = false;

  double last_dt_time_ = 0.0;
  double last_update_time_ = -1.0;
};

#endif  // WISE_2025_26_GRUPPE_C_STATE_ESTIMATION_CORE_HPP
