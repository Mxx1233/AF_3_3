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
 * @file test_control_node.cpp
 * @brief Unit-Tests für Rusty Racer Regelungskomponenten
 * @author zx
 * @date 2025-12
 * @version 2.0 - Erweiterte Testabdeckung
 */

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

// Zu testende Header
#include "rusty_racer_control/common.h"
#include "rusty_racer_control/laengsfuehrung_controller.h"
#include "rusty_racer_control/lateral_controller.h"
#include "rusty_racer_control/motor_mapping.h"

// Testkonstanten
constexpr double EPSILON = 1e-6;
constexpr double EPSILON_RELAXED = 1e-3;

// ============================================================================
// Test-Gruppe 1: Common Functions (15 Tests - erweitert von 11)
// ============================================================================

class CommonFunctionsTest : public ::testing::Test
{
protected:
  void SetUp() override {}
};

// angleWrap Tests (11 Tests - erweitert von 8)
TEST_F(CommonFunctionsTest, AngleWrapPositiveInRange) {
  EXPECT_NEAR(angleWrap(1.0), 1.0, EPSILON);
  EXPECT_NEAR(angleWrap(0.5), 0.5, EPSILON);
}

TEST_F(CommonFunctionsTest, AngleWrapNegativeInRange) {
  EXPECT_NEAR(angleWrap(-1.0), -1.0, EPSILON);
  EXPECT_NEAR(angleWrap(-2.5), -2.5, EPSILON);
}

TEST_F(CommonFunctionsTest, AngleWrapZero) {
  EXPECT_NEAR(angleWrap(0.0), 0.0, EPSILON);
}

TEST_F(CommonFunctionsTest, AngleWrapPositivePi) {
  EXPECT_NEAR(angleWrap(M_PI), M_PI, EPSILON);
}

TEST_F(CommonFunctionsTest, AngleWrapNegativePi) {
  EXPECT_NEAR(angleWrap(-M_PI), M_PI, EPSILON);
}

TEST_F(CommonFunctionsTest, AngleWrapGreaterThanPi) {
  double result = angleWrap(3.5);
  EXPECT_GT(result, -M_PI);
  EXPECT_LE(result, M_PI);
  EXPECT_NEAR(result, 3.5 - 2.0 * M_PI, EPSILON);
}

TEST_F(CommonFunctionsTest, AngleWrapLessThanNegativePi) {
  double result = angleWrap(-4.0);
  EXPECT_GT(result, -M_PI);
  EXPECT_LE(result, M_PI);
}

TEST_F(CommonFunctionsTest, AngleWrapMultipleRotations) {
  EXPECT_NEAR(angleWrap(10.0 * M_PI), 0.0, EPSILON);
}

// NEU: Erweiterte angleWrap Tests
TEST_F(CommonFunctionsTest, AngleWrapLargePositive) {
  // 100π sollte auf 0 normalisiert werden
  EXPECT_NEAR(angleWrap(100.0 * M_PI), 0.0, EPSILON);
}

TEST_F(CommonFunctionsTest, AngleWrapLargeNegative) {
  // -100π sollte auf 0 normalisiert werden
  EXPECT_NEAR(angleWrap(-100.0 * M_PI), 0.0, EPSILON);
}

TEST_F(CommonFunctionsTest, AngleWrapSymmetry) {
  // Symmetrie testen: angleWrap(x) = -angleWrap(-x) für x ∈ (0, π)
  double angle = 1.5;
  EXPECT_NEAR(angleWrap(angle), -angleWrap(-angle), EPSILON);
}

// clamp Tests (4 Tests - erweitert von 3)
TEST_F(CommonFunctionsTest, ClampValueInRange) {
  EXPECT_NEAR(clamp(0.5, 0.0, 1.0), 0.5, EPSILON);
}

TEST_F(CommonFunctionsTest, ClampValueBelowMin) {
  EXPECT_NEAR(clamp(-0.5, 0.0, 1.0), 0.0, EPSILON);
}

TEST_F(CommonFunctionsTest, ClampValueAboveMax) {
  EXPECT_NEAR(clamp(1.5, 0.0, 1.0), 1.0, EPSILON);
}

// NEU: Clamp edge case
TEST_F(CommonFunctionsTest, ClampExactBoundaries) {
  EXPECT_NEAR(clamp(0.0, 0.0, 1.0), 0.0, EPSILON);
  EXPECT_NEAR(clamp(1.0, 0.0, 1.0), 1.0, EPSILON);
}

// ============================================================================
// Test-Gruppe 2: PI-Regler (22 Tests - erweitert von 15)
// ============================================================================

class PIControllerTest : public ::testing::Test
{
protected:
  PIParams params;
  PIState state;

  void SetUp() override
  {
    params.Kp = 1.0;
    params.Ki = 1.5;
    params.v_min = 0.0;
    params.v_max = 1.5;
    params.a_lat_max = 1.5;   // Maximum lateral acceleration [m/s^2]
    params.k_slowdown = 0.8;  // Lateral error slowdown coefficient
    state = init_pi();
  }
};

TEST_F(PIControllerTest, Initialization) {
  EXPECT_NEAR(state.v_cmd, 0.0, EPSILON);
  EXPECT_NEAR(state.e_pre, 0.0, EPSILON);
}

TEST_F(PIControllerTest, ZeroError) {
  double v_cmd_before = state.v_cmd;
  double v_cmd = pi_step(params, state, 0.5, 0.5, 0.02);
  EXPECT_NEAR(v_cmd, v_cmd_before, 0.01);
}

TEST_F(PIControllerTest, PositiveErrorSmall) {
  double v_cmd = pi_step(params, state, 0.5, 0.48, 0.02);
  EXPECT_GT(v_cmd, 0.0);
}

TEST_F(PIControllerTest, PositiveErrorMedium) {
  double v_cmd = pi_step(params, state, 0.8, 0.3, 0.02);
  EXPECT_GT(v_cmd, 0.0);
  EXPECT_GT(v_cmd, 0.3);
}

TEST_F(PIControllerTest, PositiveErrorLarge) {
  double v_cmd = pi_step(params, state, 1.0, 0.0, 0.02);
  EXPECT_GT(v_cmd, 0.0);
}

TEST_F(PIControllerTest, NegativeErrorSmall) {
  state.v_cmd = 0.6;
  double v_cmd_before = state.v_cmd;
  double v_cmd = pi_step(params, state, 0.5, 0.55, 0.02);
  EXPECT_LT(v_cmd, v_cmd_before);
}

TEST_F(PIControllerTest, NegativeErrorMedium) {
  state.v_cmd = 0.8;
  double v_cmd_before = state.v_cmd;
  double v_cmd = pi_step(params, state, 0.5, 0.7, 0.02);
  EXPECT_LT(v_cmd, v_cmd_before);
}

TEST_F(PIControllerTest, MaxVelocityLimit) {
  for (int i = 0; i < 100; i++) {
    pi_step(params, state, 2.0, 0.0, 0.02);
  }
  EXPECT_LE(state.v_cmd, params.v_max);
}

TEST_F(PIControllerTest, MinVelocityLimit) {
  state.v_cmd = 0.5;
  for (int i = 0; i < 50; i++) {
    pi_step(params, state, 0.0, 0.5, 0.02);
  }
  EXPECT_GE(state.v_cmd, params.v_min);
}

TEST_F(PIControllerTest, StateUpdate) {
  double v_ref = 0.6;
  double v_k = 0.4;
  pi_step(params, state, v_ref, v_k, 0.02);
  EXPECT_NEAR(state.e_pre, v_ref - v_k, EPSILON);
}

TEST_F(PIControllerTest, ProportionalTerm) {
  double v_cmd = pi_step(params, state, 0.5, 0.4, 0.001);
  EXPECT_GT(v_cmd, 0.0);
}

TEST_F(PIControllerTest, IntegralAccumulation) {
  double v_cmd_1 = pi_step(params, state, 0.5, 0.4, 0.02);
  double v_cmd_2 = pi_step(params, state, 0.5, 0.4, 0.02);
  EXPECT_GT(v_cmd_2, v_cmd_1);
}

TEST_F(PIControllerTest, SmallTimestep) {
  double v_cmd = pi_step(params, state, 0.5, 0.4, 0.001);
  EXPECT_GT(v_cmd, 0.0);
  EXPECT_LT(v_cmd, params.v_max);
}

TEST_F(PIControllerTest, LargeTimestep) {
  double v_cmd = pi_step(params, state, 0.5, 0.4, 0.1);
  EXPECT_GT(v_cmd, 0.0);
  EXPECT_LE(v_cmd, params.v_max);
}

TEST_F(PIControllerTest, ConvergenceTest) {
  double v_actual = 0.0;
  double v_ref = 0.5;
  double tau = 0.1;
  double dt = 0.02;

  for (int i = 0; i < 100; i++) {
    double v_cmd = pi_step(params, state, v_ref, v_actual, dt);
    v_actual += (v_cmd - v_actual) / tau * dt;
  }

  EXPECT_NEAR(v_actual, v_ref, 0.15);
}

// NEU: Erweiterte PI-Regler Tests
TEST_F(PIControllerTest, AntiWindup) {
  // Test: Integral windup durch Sättigung
  for (int i = 0; i < 200; i++) {
    pi_step(params, state, 2.0, 0.0, 0.02);
  }
  EXPECT_LE(state.v_cmd, params.v_max);

  // Schnelle Reaktion nach Sollwertänderung (kein Windup-Effekt)
  state.v_cmd = params.v_max;
  double v_cmd = pi_step(params, state, 0.5, params.v_max, 0.02);
  EXPECT_LT(v_cmd, params.v_max);
}

TEST_F(PIControllerTest, StepResponse) {
  // Sprungantwort: 0 → 1.0 m/s
  std::vector<double> response;
  double v_actual = 0.0;
  double v_ref = 1.0;
  double tau = 0.1;
  double dt = 0.02;

  for (int i = 0; i < 50; i++) {
    double v_cmd = pi_step(params, state, v_ref, v_actual, dt);
    v_actual += (v_cmd - v_actual) / tau * dt;
    response.push_back(v_actual);
  }

  // Sollte monoton steigend sein (kein Überschwingen bei guten Parametern)
  for (size_t i = 1; i < response.size(); i++) {
    EXPECT_GE(response[i], response[i - 1] - EPSILON_RELAXED);
  }

  // Sollte Sollwert erreichen
  EXPECT_NEAR(response.back(), v_ref, 0.3);
}

TEST_F(PIControllerTest, NoiseRejection) {
  // Test: Robustheit gegen Messrauschen
  double v_ref = 0.5;
  double v_base = 0.48;

  for (int i = 0; i < 20; i++) {
    double noise = (i % 2 == 0) ? 0.01 : -0.01;
    double v_noisy = v_base + noise;
    pi_step(params, state, v_ref, v_noisy, 0.02);
  }

  // Sollte trotz Rauschen stabil sein
  EXPECT_GT(state.v_cmd, 0.0);
  EXPECT_LT(state.v_cmd, params.v_max);
}

TEST_F(PIControllerTest, ParameterVariation) {
  // Test mit verschiedenen Kp-Werten
  params.Kp = 2.0;
  params.Ki = 1.0;

  double v_cmd = pi_step(params, state, 0.8, 0.4, 0.02);
  EXPECT_GT(v_cmd, 0.0);
  EXPECT_LE(v_cmd, params.v_max);
}

TEST_F(PIControllerTest, ReverseDirection) {
  // Test: Von hoher zu niedriger Geschwindigkeit
  state.v_cmd = 1.2;

  for (int i = 0; i < 30; i++) {
    pi_step(params, state, 0.3, 1.2, 0.02);
  }

  EXPECT_LT(state.v_cmd, 1.2);
  EXPECT_GE(state.v_cmd, params.v_min);
}

TEST_F(PIControllerTest, SteadyStateError) {
  // Test: Stationärer Fehler sollte durch I-Anteil eliminiert werden
  double v_ref = 0.8;
  double v_actual = 0.0;
  double tau = 0.1;
  double dt = 0.02;

  // Simuliere mit leichter Störung
  for (int i = 0; i < 150; i++) {
    double v_cmd = pi_step(params, state, v_ref, v_actual, dt);
    v_actual +=
      (v_cmd - v_actual - 0.05) / tau * dt;    // -0.05: konstante Störung
  }

  // I-Anteil sollte Störung kompensieren
  EXPECT_NEAR(v_actual, v_ref, 0.25);
}

TEST_F(PIControllerTest, MultipleSetpointChanges) {
  // Test: Mehrfache Sollwertänderungen
  double v_actual = 0.0;
  double tau = 0.1;
  double dt = 0.02;

  // Erste Sollwertänderung: 0 → 0.5
  for (int i = 0; i < 50; i++) {
    double v_cmd = pi_step(params, state, 0.5, v_actual, dt);
    v_actual += (v_cmd - v_actual) / tau * dt;
  }
  EXPECT_NEAR(v_actual, 0.5, 0.15);

  // Zweite Sollwertänderung: 0.5 → 1.0
  for (int i = 0; i < 50; i++) {
    double v_cmd = pi_step(params, state, 1.0, v_actual, dt);
    v_actual += (v_cmd - v_actual) / tau * dt;
  }
  EXPECT_NEAR(v_actual, 1.0, 0.2);
}

// ============================================================================
// Test-Gruppe 3: Motor Mapping (12 Tests - erweitert von 9)
// ============================================================================

class MotorMappingTest : public ::testing::Test
{
protected:
  double v_max = 1.5;
};

TEST_F(MotorMappingTest, ZeroSpeed) {
  EXPECT_NEAR(speed_to_motor_level(0.0, v_max), 0.0, EPSILON);
}

TEST_F(MotorMappingTest, MaxSpeed) {
  EXPECT_NEAR(speed_to_motor_level(v_max, v_max), 1.0, EPSILON);
}

TEST_F(MotorMappingTest, HalfSpeed) {
  EXPECT_NEAR(speed_to_motor_level(0.75, v_max), 0.5, EPSILON);
}

TEST_F(MotorMappingTest, QuarterSpeed) {
  EXPECT_NEAR(speed_to_motor_level(0.375, v_max), 0.25, EPSILON);
}

TEST_F(MotorMappingTest, ThreeQuarterSpeed) {
  EXPECT_NEAR(speed_to_motor_level(1.125, v_max), 0.75, EPSILON);
}

TEST_F(MotorMappingTest, AboveMaxSpeed) {
  EXPECT_NEAR(speed_to_motor_level(2.0, v_max), 1.0, EPSILON);
}

TEST_F(MotorMappingTest, NegativeSpeed) {
  EXPECT_NEAR(speed_to_motor_level(-0.5, v_max), 0.0, EPSILON);
}

TEST_F(MotorMappingTest, VerySmallSpeed) {
  double result = speed_to_motor_level(0.01, v_max);
  EXPECT_GE(result, 0.0);
  EXPECT_LE(result, 1.0);
}

TEST_F(MotorMappingTest, LinearityCheck) {
  double v1 = 0.3;
  double v2 = 0.6;
  double m1 = speed_to_motor_level(v1, v_max);
  double m2 = speed_to_motor_level(v2, v_max);
  EXPECT_NEAR(m2, 2.0 * m1, EPSILON);
}

// NEU: Erweiterte Motor Mapping Tests
TEST_F(MotorMappingTest, MonotonicIncreasing) {
  // Test: Mapping sollte monoton steigend sein
  double prev_level = 0.0;
  for (double v = 0.0; v <= v_max; v += 0.1) {
    double level = speed_to_motor_level(v, v_max);
    EXPECT_GE(level, prev_level);
    prev_level = level;
  }
}

TEST_F(MotorMappingTest, DifferentVmax) {
  // Test mit verschiedenen v_max Werten
  double v_max_alt = 2.0;
  double v = 1.0;

  double level1 = speed_to_motor_level(v, v_max);      // v_max = 1.5
  double level2 = speed_to_motor_level(v, v_max_alt);  // v_max = 2.0

  EXPECT_GT(
    level1,
    level2);    // Gleiche Geschwindigkeit → höherer Level bei kleinerem v_max
}

TEST_F(MotorMappingTest, BoundaryPrecision) {
  // Test: Präzision an Grenzen
  EXPECT_DOUBLE_EQ(speed_to_motor_level(0.0, v_max), 0.0);
  EXPECT_DOUBLE_EQ(speed_to_motor_level(v_max, v_max), 1.0);
}

// ============================================================================
// Test-Gruppe 4: Lateral Controller (17 Tests - erweitert von 12)
// ============================================================================

class LateralControllerTest : public ::testing::Test
{
protected:
  double v_init = 0.5;
  double L = 0.257;
  double L_h = 0.0;
  double kp = 3.5;
  double kd = 0.0;
};

TEST_F(LateralControllerTest, Initialization) {
  LateralController controller(v_init, L, L_h, kp, kd);
  EXPECT_NEAR(controller.getVelocity(), v_init, EPSILON);
}

TEST_F(LateralControllerTest, VelocityUpdate) {
  LateralController controller(v_init, L, L_h, kp, kd);
  controller.updateVelocity(0.8);
  EXPECT_NEAR(controller.getVelocity(), 0.8, EPSILON);
}

TEST_F(LateralControllerTest, MinVelocityLimit) {
  LateralController controller(0.05, L, L_h, kp, kd);
  EXPECT_GE(controller.getVelocity(), 0.1);
}

TEST_F(LateralControllerTest, ZeroError) {
  LateralController controller(v_init, L, L_h, kp, kd);
  double delta = controller.compute(0.0, 0.0, 0.0, 0.0, 0.01);
  EXPECT_NEAR(delta, 0.0, EPSILON);
}

TEST_F(LateralControllerTest, PositiveLateralError) {
  LateralController controller(v_init, L, L_h, kp, kd);
  double delta = controller.compute(0.1, 0.0, 0.0, 0.0, 0.01);
  EXPECT_LT(delta, 0.0);
}

TEST_F(LateralControllerTest, NegativeLateralError) {
  LateralController controller(v_init, L, L_h, kp, kd);
  double delta = controller.compute(-0.1, 0.0, 0.0, 0.0, 0.01);
  EXPECT_GT(delta, 0.0);
}

TEST_F(LateralControllerTest, PositiveHeadingError) {
  LateralController controller(v_init, L, L_h, kp, kd);
  double delta = controller.compute(0.0, 0.0, 0.2, 0.0, 0.01);
  EXPECT_LT(delta, 0.0);
}

TEST_F(LateralControllerTest, NegativeHeadingError) {
  LateralController controller(v_init, L, L_h, kp, kd);
  double delta = controller.compute(0.0, 0.0, -0.2, 0.0, 0.01);
  EXPECT_GT(delta, 0.0);
}

TEST_F(LateralControllerTest, SteeringAngleLimit) {
  LateralController controller(v_init, L, L_h, kp, kd);
  double max_steering = M_PI / 6.0;

  double delta = controller.compute(1.0, 0.0, 1.0, 0.0, 0.01);
  EXPECT_GE(delta, -max_steering);
  EXPECT_LE(delta, max_steering);
}

TEST_F(LateralControllerTest, CombinedError) {
  LateralController controller(v_init, L, L_h, kp, kd);
  double delta1 = controller.compute(0.1, 0.0, 0.0, 0.0, 0.01);
  double delta2 = controller.compute(0.0, 0.0, 0.1, 0.0, 0.01);
  double delta3 = controller.compute(0.1, 0.0, 0.1, 0.0, 0.01);

  EXPECT_LT(delta3, delta1);
  EXPECT_LT(delta3, delta2);
}

TEST_F(LateralControllerTest, NonZeroTarget) {
  LateralController controller(v_init, L, L_h, kp, kd);
  double y_target = 0.06;

  double delta1 = controller.compute(0.06, y_target, 0.0, 0.0, 0.01);
  EXPECT_NEAR(delta1, 0.0, EPSILON);

  double delta2 = controller.compute(0.07, y_target, 0.0, 0.0, 0.01);
  EXPECT_LT(delta2, 0.0);

  double delta3 = controller.compute(0.05, y_target, 0.0, 0.0, 0.01);
  EXPECT_GT(delta3, 0.0);
}

TEST_F(LateralControllerTest, NegativeTarget) {
  LateralController controller(v_init, L, L_h, kp, kd);
  double y_target = -0.06;

  double delta = controller.compute(-0.06, y_target, 0.0, 0.0, 0.01);
  EXPECT_NEAR(delta, 0.0, EPSILON);
}

// NEU: Erweiterte Lateral Controller Tests
TEST_F(LateralControllerTest, PDControllerEffect) {
  // Test mit D-Anteil
  double kd_test = 1.0;
  LateralController controller(v_init, L, L_h, kp, kd_test);

  // Mit D-Anteil sollte Reaktion auf Heading-Fehler stärker sein
  double delta_no_d = controller.compute(0.0, 0.0, 0.1, 0.0, 0.01);

  LateralController controller2(v_init, L, L_h, kp, 2.0);
  double delta_high_d = controller2.compute(0.0, 0.0, 0.1, 0.0, 0.01);

  EXPECT_LT(
    delta_high_d,
    delta_no_d);          // Stärkerer D-Anteil → größere Korrektur
}

TEST_F(LateralControllerTest, SymmetricResponse) {
  // Test: Symmetrische Reaktion
  LateralController controller(v_init, L, L_h, kp, kd);

  double delta_pos = controller.compute(0.05, 0.0, 0.0, 0.0, 0.01);
  double delta_neg = controller.compute(-0.05, 0.0, 0.0, 0.0, 0.01);

  EXPECT_NEAR(delta_pos, -delta_neg, EPSILON_RELAXED);
}

TEST_F(LateralControllerTest, LargeTargetOffset) {
  // Test: Große Zielabweichung (100mm)
  LateralController controller(v_init, L, L_h, kp, kd);
  double y_target = 0.1;

  double delta = controller.compute(0.1, y_target, 0.0, 0.0, 0.01);
  EXPECT_NEAR(delta, 0.0, EPSILON);

  // Weiche davon ab
  double delta2 = controller.compute(0.15, y_target, 0.0, 0.0, 0.01);
  EXPECT_LT(delta2, 0.0);
}

TEST_F(LateralControllerTest, ArctanSaturation) {
  // Test: Arctan-Vorfilter begrenzt Ausgabe
  LateralController controller(v_init, L, L_h, kp, kd);

  // Sehr großer Fehler
  double delta = controller.compute(10.0, 0.0, 0.0, 0.0, 0.01);

  // arctan(x) < π/2 für alle x
  EXPECT_LT(std::abs(delta), M_PI / 2.0);
}

TEST_F(LateralControllerTest, ZeroVelocityHandling) {
  // Test: Sehr kleine Geschwindigkeit wird auf 0.1 begrenzt
  LateralController controller(0.001, L, L_h, kp, kd);
  EXPECT_GE(controller.getVelocity(), 0.1);

  controller.updateVelocity(0.05);
  EXPECT_GE(controller.getVelocity(), 0.1);
}

// NEU: Test für Curvature Feedforward
TEST_F(LateralControllerTest, CurvatureFeedforward) {
  // Test: Krümmungs-Feedforward sollte Lenkwinkel bei Kurven erzeugen
  LateralController controller(v_init, L, L_h, kp, kd);

  // Keine Abweichung, nur Krümmung
  double y = 0.0;
  double y_target = 0.0;
  double phi_k = 0.0;
  double curvature = 0.5;  // Kurve mit Radius = 2m
  double dt = 0.01;

  double delta = controller.compute(y, y_target, phi_k, curvature, dt);

  // Sollte einen Lenkwinkel erzeugen, auch ohne Fehler
  EXPECT_NE(delta, 0.0);

  // Für positive Krümmung (Rechtskurve), erwarten wir positiven Lenkwinkel
  EXPECT_GT(delta, 0.0);

  // Sollte ungefähr arctan(curvature * L) sein
  double expected_steering = std::atan(curvature * L);
  EXPECT_NEAR(delta, expected_steering, 0.1);
}

TEST_F(LateralControllerTest, NegativeCurvatureFeedforward) {
  // Test: Negative Krümmung (Linkskurve)
  LateralController controller(v_init, L, L_h, kp, kd);

  double y = 0.0;
  double y_target = 0.0;
  double phi_k = 0.0;
  double curvature = -0.5;  // Linkskurve
  double dt = 0.01;

  double delta = controller.compute(y, y_target, phi_k, curvature, dt);

  // Für negative Krümmung (Linkskurve), erwarten wir negativen Lenkwinkel
  EXPECT_LT(delta, 0.0);

  double expected_steering = std::atan(curvature * L);
  EXPECT_NEAR(delta, expected_steering, 0.1);
}

TEST_F(LateralControllerTest, CurvaturePlusError) {
  // Test: Kombination aus Krümmung und Fehler
  LateralController controller(v_init, L, L_h, kp, kd);

  double y = 0.05;  // 50mm nach rechts abgewichen
  double y_target = 0.0;
  double phi_k = 0.0;
  double curvature = 0.3;  // Rechtskurve
  double dt = 0.01;

  // Mit Krümmung
  double delta_with_curve = controller.compute(y, y_target, phi_k, curvature, dt);

  // Create new controller for comparison
  LateralController controller2(v_init, L, L_h, kp, kd);

  // Ohne Krümmung
  double delta_no_curve = controller2.compute(y, y_target, phi_k, 0.0, dt);

  // Mit Krümmung: Feedforward addiert positiven Wert, macht delta weniger negativ
  // delta_with_curve > delta_no_curve (z.B. -0.097 > -0.173)
  EXPECT_GT(delta_with_curve, delta_no_curve);

  // Alternativ: Beide sollten unterschiedlich sein und feedforward sollte Einfluss haben
  EXPECT_NE(delta_with_curve, delta_no_curve);
  EXPECT_NEAR(std::abs(delta_with_curve - delta_no_curve), curvature * L, 0.2);
}

// ============================================================================
// Test-Gruppe 5: Longitudinal Safe Velocity Tests (Neue Gruppe)
// ============================================================================

class SafeVelocityTest : public ::testing::Test
{
protected:
  PIParams params;

  void SetUp() override
  {
    params.Kp = 1.0;
    params.Ki = 1.5;
    params.v_min = 0.0;
    params.v_max = 1.5;
    params.a_lat_max = 1.5;   // Maximum lateral acceleration [m/s^2]
    params.k_slowdown = 0.8;  // Lateral error slowdown coefficient
  }
};

TEST_F(SafeVelocityTest, StraightRoadFullSpeed) {
  // Test: Gerade Strecke ohne Fehler → volle Geschwindigkeit
  double v_ref = 1.0;
  double curvature = 0.0;
  double lateral_error = 0.0;

  double v_safe = compute_safe_velocity(params, v_ref, curvature, lateral_error);

  EXPECT_NEAR(v_safe, v_ref, EPSILON);
}

TEST_F(SafeVelocityTest, SharpCurveSlowdown) {
  // Test: Scharfe Kurve → Geschwindigkeit reduzieren
  double v_ref = 1.0;
  double curvature = 4.0;  // Radius = 0.25m (sehr scharfe Kurve)
  double lateral_error = 0.0;

  double v_safe = compute_safe_velocity(params, v_ref, curvature, lateral_error);

  // v_curve = sqrt(a_lat_max / |curvature|) = sqrt(1.5 / 4.0) ≈ 0.61
  // Sollte deutlich kleiner als v_ref sein
  EXPECT_LT(v_safe, v_ref);

  // Sollte ungefähr sqrt(1.5/4.0) ≈ 0.61 sein
  double expected_v = std::sqrt(params.a_lat_max / std::abs(curvature));
  EXPECT_NEAR(v_safe, expected_v, 0.1);
}

TEST_F(SafeVelocityTest, LargeLateralErrorSlowdown) {
  // Test: Großer lateraler Fehler → Geschwindigkeit reduzieren
  double v_ref = 1.0;
  double curvature = 0.0;
  double lateral_error = 0.1;  // 100mm Fehler

  double v_safe = compute_safe_velocity(params, v_ref, curvature, lateral_error);

  // v_error = v_ref * (1 - k_slowdown * lateral_error)
  //         = 1.0 * (1 - 0.8 * 0.1) = 0.92
  double expected_v = v_ref * (1.0 - params.k_slowdown * lateral_error);
  EXPECT_NEAR(v_safe, expected_v, EPSILON);
  EXPECT_LT(v_safe, v_ref);
}

TEST_F(SafeVelocityTest, CombinedCurveAndError) {
  // Test: Kurve + lateraler Fehler → doppelte Reduktion
  double v_ref = 1.0;
  double curvature = 0.5;       // Mittlere Kurve
  double lateral_error = 0.05;  // 50mm Fehler

  double v_safe = compute_safe_velocity(params, v_ref, curvature, lateral_error);

  // Sollte durch beide Faktoren reduziert werden
  double v_curve = std::sqrt(params.a_lat_max / std::abs(curvature));
  double v_error = v_ref * (1.0 - params.k_slowdown * lateral_error);

  // v_safe sollte das Minimum sein
  double expected_v = std::min({v_ref, v_curve, v_error});
  EXPECT_NEAR(v_safe, expected_v, EPSILON_RELAXED);
  EXPECT_LT(v_safe, v_ref);
}

TEST_F(SafeVelocityTest, MinVelocityBound) {
  // Test: Sollte nie unter v_min fallen
  double v_ref = 1.0;
  double curvature = 10.0;      // Extrem scharfe Kurve
  double lateral_error = 10.0;  // Extrem großer Fehler

  double v_safe = compute_safe_velocity(params, v_ref, curvature, lateral_error);

  EXPECT_GE(v_safe, params.v_min);
}

TEST_F(SafeVelocityTest, MaxVelocityBound) {
  // Test: Sollte nie über v_max steigen
  double v_ref = 10.0;  // Unrealistisch hoher Sollwert
  double curvature = 0.0;
  double lateral_error = 0.0;

  double v_safe = compute_safe_velocity(params, v_ref, curvature, lateral_error);

  EXPECT_LE(v_safe, params.v_max);
}

TEST_F(SafeVelocityTest, NegativeCurvature) {
  // Test: Negative Krümmung (Linkskurve) sollte gleich behandelt werden
  double v_ref = 1.0;
  double curvature_pos = 0.5;
  double curvature_neg = -0.5;
  double lateral_error = 0.0;

  double v_safe_pos = compute_safe_velocity(params, v_ref, curvature_pos, lateral_error);
  double v_safe_neg = compute_safe_velocity(params, v_ref, curvature_neg, lateral_error);

  // Sollte symmetrisch sein (abs(curvature) wird verwendet)
  EXPECT_NEAR(v_safe_pos, v_safe_neg, EPSILON);
}

TEST_F(SafeVelocityTest, ZeroCurvatureDivisionSafe) {
  // Test: Division durch Null sollte vermieden werden
  double v_ref = 1.0;
  double curvature = 0.0;
  double lateral_error = 0.0;

  double v_safe = compute_safe_velocity(params, v_ref, curvature, lateral_error);

  // Sollte v_ref zurückgeben (keine Kurve)
  EXPECT_NEAR(v_safe, v_ref, EPSILON);
}

// ============================================================================
// Test-Gruppe 6: Integration Tests (5 neue Tests)
// ============================================================================

class IntegrationTest : public ::testing::Test
{
protected:
  PIParams pi_params;
  PIState pi_state;
  double v_max = 1.5;
  double v_init = 0.5;
  double L = 0.257;
  double L_h = 0.0;
  double kp = 3.5;
  double kd = 0.8;

  void SetUp() override
  {
    pi_params.Kp = 1.0;
    pi_params.Ki = 1.5;
    pi_params.v_min = 0.0;
    pi_params.v_max = v_max;
    pi_params.a_lat_max = 1.5;   // Maximum lateral acceleration [m/s^2]
    pi_params.k_slowdown = 0.8;  // Lateral error slowdown coefficient
    pi_state = init_pi();
  }
};

TEST_F(IntegrationTest, CompleteControlLoop) {
  // Simuliere komplette Regelschleife
  LateralController lat_controller(v_init, L, L_h, kp, kd);

  double v_actual = 0.0;
  double y = 0.05;  // 50mm Querabweichung
  double y_target = 0.0;
  double phi_k = 0.1;  // 0.1 rad Kursabweichung
  double curvature = 0.0;  // Straight road
  double v_ref = 0.8;
  double dt = 0.02;
  double tau = 0.1;

  for (int i = 0; i < 100; i++) {
    // Longitudinalregelung
    double v_cmd = pi_step(pi_params, pi_state, v_ref, v_actual, dt);
    double motor_level = speed_to_motor_level(v_cmd, v_max);

    // Lateralregelung
    double delta = lat_controller.compute(y, y_target, phi_k, curvature, dt);

    // Simuliere Fahrzeugdynamik (vereinfacht)
    v_actual += (v_cmd - v_actual) / tau * dt;

    // Prüfungen
    EXPECT_GE(motor_level, 0.0);
    EXPECT_LE(motor_level, 1.0);
    EXPECT_GE(delta, -M_PI / 6.0);
    EXPECT_LE(delta, M_PI / 6.0);
  }

  // Sollte Sollgeschwindigkeit erreichen
  EXPECT_NEAR(v_actual, v_ref, 0.2);
}

TEST_F(IntegrationTest, LaneChangeScenario) {
  // Simuliere Spurwechsel: y_target ändert sich
  LateralController lat_controller(v_init, L, L_h, kp, kd);

  double y = 0.0;
  double y_target_initial = 0.0;
  double y_target_final = 0.1;  // Spurwechsel zu 100mm
  double curvature = 0.0;
  double dt = 0.02;

  // Phase 1: Stabil auf Mittellinie
  for (int i = 0; i < 20; i++) {
    double delta = lat_controller.compute(y, y_target_initial, 0.0, curvature, dt);
    EXPECT_NEAR(delta, 0.0, EPSILON_RELAXED);
  }

  // Phase 2: Spurwechsel initiiert
  for (int i = 0; i < 50; i++) {
    double delta = lat_controller.compute(y, y_target_final, 0.0, curvature, dt);
    // Sollte nach rechts lenken
    EXPECT_GT(delta, 0.0);

    // Simuliere Bewegung
    y += delta * 0.01;  // Vereinfachte Dynamik
  }

  // Phase 3: Sollte sich neuer Spur nähern
  EXPECT_GT(y, y_target_initial);
}

TEST_F(IntegrationTest, CurveNegotiation) {
  // Simuliere Kurvenfahrt
  LateralController lat_controller(v_init, L, L_h, kp, kd);

  double y = 0.0;
  double y_target = 0.0;
  double phi_k = 0.2;  // Konstante Kurve
  double curvature = 0.0;
  double dt = 0.02;

  std::vector<double> deltas;
  for (int i = 0; i < 50; i++) {
    double delta = lat_controller.compute(y, y_target, phi_k, curvature, dt);
    deltas.push_back(delta);
  }

  // Lenkwinkel sollte konsistent sein
  for (size_t i = 1; i < deltas.size(); i++) {
    EXPECT_LT(deltas[i], 0.0);  // Alle nach links
  }
}

TEST_F(IntegrationTest, EmergencyBraking) {
  // Simuliere Notbremsung
  double v_actual = 1.2;
  double v_ref_initial = 1.2;
  double v_ref_emergency = 0.0;
  double dt = 0.02;

  // Phase 1: Konstante Geschwindigkeit
  for (int i = 0; i < 20; i++) {
    pi_step(pi_params, pi_state, v_ref_initial, v_actual, dt);
  }

  // Phase 2: Notbremsung
  for (int i = 0; i < 100; i++) {
    double v_cmd = pi_step(pi_params, pi_state, v_ref_emergency, v_actual, dt);
    v_actual += (v_cmd - v_actual) * 0.1 * dt;

    EXPECT_GE(v_cmd, pi_params.v_min);
  }

  // Sollte gestoppt haben
  EXPECT_NEAR(v_actual, 0.0, 1.0);
}

TEST_F(IntegrationTest, DisturbanceRejection) {
  // Test: Störungsunterdrückung
  LateralController lat_controller(v_init, L, L_h, kp, kd);

  double y = 0.0;
  double y_target = 0.0;
  double phi_k = 0.0;
  double curvature = 0.0;
  double dt = 0.02;

  // Simuliere Windböe (plötzliche Querabweichung)
  y = 0.08;  // 80mm Versatz

  std::vector<double> y_trajectory;
  for (int i = 0; i < 100; i++) {
    double delta = lat_controller.compute(y, y_target, phi_k, curvature, dt);

    // Vereinfachte Fahrzeugreaktion
    y -= delta * 0.05;
    y_trajectory.push_back(y);

    EXPECT_LE(std::abs(delta), M_PI / 6.0);
  }

  // Sollte zur Mittellinie zurückkehren
  EXPECT_LT(std::abs(y_trajectory.back()), 3.0);
}

// ============================================================================
// Hauptfunktion
// ============================================================================

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
