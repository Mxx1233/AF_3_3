/**
 * @file common.h
 * @brief Hilfsfunktionen für Regelung
 * @author zx
 * @date 2025-12
 */

#pragma once
#include <cmath>

/**
 * @brief Normalisiert Winkel auf Bereich (-π, π]
 * @param a Winkel [rad]
 * @return Normalisierter Winkel [rad]
 */
inline double angleWrap(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a <= -M_PI) {a += 2.0 * M_PI;}
  return a;
}

/**
 * @brief Begrenzt Wert auf Bereich [min, max]
 * @param x Eingangswert
 * @param val_min Minimalwert
 * @param val_max Maximalwert
 * @return Begrenzter Wert
 */
inline double clamp(double x, double val_min, double val_max)
{
  if (x < val_min) {return val_min;}
  if (x > val_max) {return val_max;}
  return x;
}
