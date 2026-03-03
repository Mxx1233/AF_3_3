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
//  * @file laengsfuehrung_controller.h
//  * @brief PI-Geschwindigkeitsregler (Geschwindigkeitsdomäne)
//  * @author zx
//  * @date 2025-12
//  */

// #pragma once
// #include <algorithm>
// #include <cmath>

// /**
//  * @struct PIParams
//  * @brief PI-Reglerparameter
//  */
// struct PIParams
// {
//   double Kp = 1.0;     // Proportionalverstärkung (empfohlen: 0.5 - 2.0)
//   double Ki = 1.5;     // Integralverstärkung (empfohlen: 0.5 - 2.0)
//   double v_min = 0.0;  // Minimale Geschwindigkeit [m/s]
//   double v_max = 1.5;  // Maximale Geschwindigkeit [m/s] (Messung erforderlich)
//   double a_lat_max = 1.5;  // Maximale laterale Beschleunigung [m/s^2]
//   double k_slowdown = 0.8;  // Verlangsamungsfaktor für laterale Fehler
// };

// /**
//  * @struct PIState
//  * @brief PI-Reglerzustand
//  */
// struct PIState
// {
//   double v_cmd = 0.0;  // Aktueller Geschwindigkeitsbefehl [m/s]
//   double e_pre = 0.0;  // Vorheriger Geschwindigkeitsfehler [m/s]
// };

// /**
//  * @brief Initialisiert PI-Regler
//  * @return Initialisierter Zustand
//  */
// inline PIState init_pi()
// {
//   PIState s;
//   s.v_cmd = 0.0;
//   s.e_pre = 0.0;
//   return s;
// }

// /**
//  * @brief PI-Geschwindigkeitsregler - Inkrementelle PI-Berechnung
//  * @param p Reglerparameter
//  * @param s Reglerzustand (wird modifiziert)
//  * @param v_ref Sollgeschwindigkeit [m/s]
//  * @param v_k Aktuelle Geschwindigkeit [m/s]
//  * @param dt Abtastzeit [s]
//  * @return v_cmd Geschwindigkeitsbefehl [m/s]
//  *
//  * Algorithmus:
//  *   1. Fehler berechnen: e_k = v_ref - v_k
//  *   2. Inkrementelles PI: Δv = Kp·(e_k - e_pre) + Ki·dt·e_k
//  *   3. Befehl aktualisieren: v_cmd = v_cmd_pre + Δv
//  *   4. Begrenzen auf [v_min, v_max]
//  */
// inline double pi_step(
//   const PIParams & p, PIState & s, double v_ref, double v_k,
//   double dt)
// {
//   // Fehler berechnen
//   double e_k = v_ref - v_k;

//   // Inkrementelles PI
//   double delta_v = p.Kp * (e_k - s.e_pre) + p.Ki * dt * e_k;

//   // Befehl aktualisieren
//   s.v_cmd += delta_v;

//   // Begrenzung
//   s.v_cmd = std::max(p.v_min, std::min(s.v_cmd, p.v_max));

//   // Zustand speichern
//   s.e_pre = e_k;

//   return s.v_cmd;
// }

// /**
//  * @brief Berechnet sichere Geschwindigkeit basierend auf Krümmung und lateralem Fehler
//  * @param p Reglerparameter
//  * @param v_ref Sollgeschwindigkeit [m/s]
//  * @param curvature Straßenkrümmung [1/m]
//  * @param lateral_error Absoluter lateraler Fehler [m]
//  * @return v_safe Sichere Geschwindigkeit [m/s]
//  *
//  * Algorithmus:
//  *   1. Berechne maximale Geschwindigkeit für Kurve: v_curve = sqrt(a_lat_max / |curvature|)
//  *   2. Reduziere Geschwindigkeit bei lateralem Fehler:
//  *      v_error = v_ref * max(0, 1 - k_slowdown * |lateral_error|)
//  *   3. Rückgabe: min(v_ref, v_curve, v_error), begrenzt auf [v_min, v_max]
//  */
// inline double compute_safe_velocity(
//   const PIParams & p, double v_ref, double curvature,
//   double lateral_error)
// {
//   // 1. Kurvengeschwindigkeit basierend auf Krümmung
//   double v_curve = v_ref;
//   if (std::abs(curvature) > 1e-6) {  // Avoid division by zero
//     v_curve = std::sqrt(p.a_lat_max / std::abs(curvature));
//   }

//   // 2. Verlangsamung bei lateralem Fehler (mit Schutz vor negativen Werten)
//   double v_error = v_ref * std::max(0.0, 1.0 - p.k_slowdown * lateral_error);

//   // 3. Minimum der drei Geschwindigkeiten
//   double v_safe = std::min({v_ref, v_curve, v_error});

//   // Begrenzung auf [v_min, v_max]
//   v_safe = std::max(p.v_min, std::min(v_safe, p.v_max));

//   return v_safe;
// }

// /**
//  * @brief Gibt PI-Reglerzustand als String zurück (für Debugging)
//  * @param s Reglerzustand
//  * @param v_ref Sollgeschwindigkeit [m/s]
//  * @param v_k Aktuelle Geschwindigkeit [m/s]
//  * @return Formatierter Zustandsstring
//  */
// inline const char * pi_state_string(const PIState & s, double v_ref, double v_k)
// {
//   static char buffer[256];
//   snprintf(
//     buffer, sizeof(buffer),
//     "PI-Zustand: v_cmd=%.3f m/s, e_aktuell=%.3f, e_pre=%.3f, "
//     "v_soll=%.3f, v_ist=%.3f",
//     s.v_cmd, (v_ref - v_k), s.e_pre, v_ref, v_k);
//   return buffer;
// }


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
 * @file laengsfuehrung_controller.h
 * @brief PI-Geschwindigkeitsregler (Geschwindigkeitsdomäne)
 * @author zx
 * @date 2025-12
 */

#pragma once
#include <algorithm>

/**
 * @struct PIParams
 * @brief PI-Reglerparameter
 */
struct PIParams
{
  double Kp = 1.0;     // Proportionalverstärkung (empfohlen: 0.5 - 2.0)
  double Ki = 1.5;     // Integralverstärkung (empfohlen: 0.5 - 2.0)
  double v_min = 0.0;  // Minimale Geschwindigkeit [m/s]
  double v_max = 1.5;  // Maximale Geschwindigkeit [m/s] (Messung erforderlich)
};

/**
 * @struct PIState
 * @brief PI-Reglerzustand
 */
struct PIState
{
  double v_cmd = 0.0;  // Aktueller Geschwindigkeitsbefehl [m/s]
  double e_pre = 0.0;  // Vorheriger Geschwindigkeitsfehler [m/s]
};

/**
 * @brief Initialisiert PI-Regler
 * @return Initialisierter Zustand
 */
inline PIState init_pi()
{
  PIState s;
  s.v_cmd = 0.0;
  s.e_pre = 0.0;
  return s;
}

/**
 * @brief PI-Geschwindigkeitsregler - Inkrementelle PI-Berechnung
 * @param p Reglerparameter
 * @param s Reglerzustand (wird modifiziert)
 * @param v_ref Sollgeschwindigkeit [m/s]
 * @param v_k Aktuelle Geschwindigkeit [m/s]
 * @param dt Abtastzeit [s]
 * @return v_cmd Geschwindigkeitsbefehl [m/s]
 *
 * Algorithmus:
 *   1. Fehler berechnen: e_k = v_ref - v_k
 *   2. Inkrementelles PI: Δv = Kp·(e_k - e_pre) + Ki·dt·e_k
 *   3. Befehl aktualisieren: v_cmd = v_cmd_pre + Δv
 *   4. Begrenzen auf [v_min, v_max]
 */
inline double pi_step(
  const PIParams & p, PIState & s, double v_ref, double v_k,
  double dt)
{
  // Fehler berechnen
  double e_k = v_ref - v_k;

  // Inkrementelles PI
  double delta_v = p.Kp * (e_k - s.e_pre) + p.Ki * dt * e_k;

  // Befehl aktualisieren
  s.v_cmd += delta_v;

  // Begrenzung
  s.v_cmd = std::max(p.v_min, std::min(s.v_cmd, p.v_max));

  // Zustand speichern
  s.e_pre = e_k;

  return s.v_cmd;
}

/**
 * @brief Gibt PI-Reglerzustand als String zurück (für Debugging)
 * @param s Reglerzustand
 * @param v_ref Sollgeschwindigkeit [m/s]
 * @param v_k Aktuelle Geschwindigkeit [m/s]
 * @return Formatierter Zustandsstring
 */
inline const char * pi_state_string(const PIState & s, double v_ref, double v_k)
{
  static char buffer[256];
  snprintf(
    buffer, sizeof(buffer),
    "PI-Zustand: v_cmd=%.3f m/s, e_aktuell=%.3f, e_pre=%.3f, "
    "v_soll=%.3f, v_ist=%.3f",
    s.v_cmd, (v_ref - v_k), s.e_pre, v_ref, v_k);
  return buffer;
}
