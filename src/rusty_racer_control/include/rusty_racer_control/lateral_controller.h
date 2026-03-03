// Copyright 2025 Rusty Racer Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// /**
//  * @file lateral_controller.h
//  * @brief Lateraler PD-Regler mit Curvature Feedforward
//  * @author zx
//  * @date 2025-01
//  */

// #pragma once
// #include <algorithm>
// #include <cmath>

// /**
//  * @class LateralController
//  * @brief Lateraler Regler für Spurführung
//  *
//  * Regelgesetz (tan-Domäne):
//  * φ*L_feedback = -kp·e_y - (kp+kd)·φK
//  * φ*L_feedforward = κ·l  [Dokumentformel 3.4]
//  * φ*L = φ*L_feedback + φ*L_feedforward
//  *
//  * Vorfilter (Umrechnung in Winkel-Domäne):
//  * δ = arctan(φ*L)
//  */
// class LateralController {
// public:
//   /**
//    * @brief Konstruktor
//    * @param v Geschwindigkeit [m/s]
//    * @param l Radstand [m]
//    * @param l_h Abstand Sensorpunkt-Hinterachse [m]
//    * @param kp Proportionalverstärkung
//    * @param kd Differentialverstärkung
//    */
//   LateralController(double v, double l, double l_h, double kp, double kd)
//     : v_(v), l_(l), l_h_(l_h), kp_(kp), kd_(kd) {
//     if (v_ < 0.1) {
//       v_ = 0.1;  // Mindestgeschwindigkeit
//     }
//   }

//   /**
//    * @brief Aktualisiert die Geschwindigkeit
//    * @param v Neue Geschwindigkeit [m/s]
//    */
//   void updateVelocity(double v) {v_ = std::max(0.1, v);}

//   /**
//    * @brief Berechnet Lenkwinkel aus Querabweichung und Kurswinkel
//    * @param y Aktuelle Querabweichung [m]
//    * @param y_target Ziel-Querabweichung [m]
//    * @param phi_k Kursabweichung [rad]
//    * @param curvature Straßenkrümmung [1/m]
//    * @param dt Zeitschritt [s] - unused for PD controller
//    * @return delta Lenkwinkel [rad]
//    */
//   double compute(double y, double y_target, double phi_k, double curvature, double dt)
//   {
//     (void)dt;  // Unused parameter for PD controller

//     // Lateraler Fehler berechnen
//     double e_y = y - y_target;

//     // Regler in tan-Domäne (linearisierter Bereich)
//     // Feedback: φ*L_feedback = -kp·e_y - (kp+kd)·φK
//     double phi_L_star_feedback = -kp_ * e_y - (kp_ + kd_) * phi_k;

//     // Feedforward: φ*L_feedforward = κ·l  [Formel 3.4: κ·l = tan(φL)]
//     double phi_L_star_feedforward = curvature * l_;

//     // Gesamte Stellgröße in tan-Domäne
//     double phi_L_star = phi_L_star_feedback + phi_L_star_feedforward;

//     // Begrenzung in tan-Domäne (±30° → tan(±30°) ≈ ±0.577)
//     const double max_phi_L_star = std::tan(M_PI / 6.0);
//     phi_L_star = std::clamp(phi_L_star, -max_phi_L_star, max_phi_L_star);

//     // Arctan-Vorfilter: φL = arctan(φ*L)
//     double steering_angle = std::atan(phi_L_star);

//     return steering_angle;
//   }

//   /**
//    * @brief Gibt aktuelle Geschwindigkeit zurück
//    * @return Geschwindigkeit [m/s]
//    */
//   double getVelocity() const {return v_;}

// private:
//   double v_;    // Geschwindigkeit [m/s]
//   double l_;    // Radstand [m]
//   double l_h_;  // Abstand Sensor-Hinterachse [m]
//   double kp_;   // P-Verstärkung
//   double kd_;   // D-Verstärkung
// };


// Copyright 2025 Rusty Racer Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file lateral_controller.h
 * @brief Lateraler PD-Regler mit Vorfilter
 * @author zx
 * @date 2025-12
 */

#pragma once
#include <algorithm>
#include <cmath>

/**
 * @class LateralController
 * @brief Lateraler Regler für Spurführung
 *
 * Regelgesetz: φL* = -kp·e_y - (kp+kd)·φK, wobei e_y = y - y_target
 * Vorfilter: δ = arctan(φL*)
 */
class LateralController {
public:
  /**
   * @brief Konstruktor
   * @param v Geschwindigkeit [m/s]
   * @param l Radstand [m]
   * @param l_h Abstand Sensorpunkt-Hinterachse [m]
   * @param kp Proportionalverstärkung
   * @param kd Differentialverstärkung
   */
  LateralController(double v, double l, double l_h, double kp, double kd)
    : v_(v), l_(l), l_h_(l_h), kp_(kp), kd_(kd) {
    if (v_ < 0.1) {
      v_ = 0.1;               // Mindestgeschwindigkeit
    }
  }

  /**
   * @brief Aktualisiert die Geschwindigkeit
   * @param v Neue Geschwindigkeit [m/s]
   */
  void updateVelocity(double v) {v_ = std::max(0.1, v);}

  /**
   * @brief Berechnet Lenkwinkel aus Querabweichung und Kurswinkel
   * @param y Aktuelle Querabweichung [m]
   * @param y_target Ziel-Querabweichung [m] (target lateral position to maintain)
   * @param phi_k Kursabweichung [rad]
   * @return delta Lenkwinkel [rad]
   */
  double compute(double y, double y_target, double phi_k)
  {
    // Lateraler Fehler berechnen
    double e_y = y - y_target;

    // PD-Regelung
    double p_term = -kp_ * e_y - kp_ * phi_k;
    double d_term = -kd_ * phi_k;
    double steering_input = p_term + d_term;

    // Begrenzung auf ±30°
    const double max_steering = M_PI / 6.0;
    steering_input =
      std::max(-max_steering, std::min(steering_input, max_steering));

    // Arctan-Vorfilter
    double steering_angle = std::atan(steering_input);

    return steering_angle;
  }

  /**
   * @brief Gibt aktuelle Geschwindigkeit zurück
   * @return Geschwindigkeit [m/s]
   */
  double getVelocity() const {return v_;}

private:
  double v_;    // Geschwindigkeit [m/s]
  double l_;    // Radstand [m]
  double l_h_;  // Abstand Sensor-Hinterachse [m]
  double kp_;   // P-Verstärkung
  double kd_;   // D-Verstärkung
};
