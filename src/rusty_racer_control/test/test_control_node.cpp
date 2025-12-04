/**
 * @file test_control_node.cpp
 * @brief Unit-Tests für Rusty Racer Regelungskomponenten
 * @author zx
 * @date 2025-12
 */

#include <gtest/gtest.h>
#include <cmath>

// Zu testende Header
#include "rusty_racer_control/common.h"
#include "rusty_racer_control/laengsfuehrung_controller.h"
#include "rusty_racer_control/motor_mapping.h"
#include "rusty_racer_control/lateral_controller.h"

// Testkonstanten
constexpr double EPSILON = 1e-6;

// ============================================================================
// Test-Gruppe 1: Common Functions (11 Tests)
// ============================================================================

class CommonFunctionsTest : public ::testing::Test {
protected:
    void SetUp() override {}
};

// angleWrap Tests (8 Tests)
TEST_F(CommonFunctionsTest, AngleWrapPositiveInRange) {
    // Winkel im Bereich bleiben unverändert
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
    // π bleibt π
    EXPECT_NEAR(angleWrap(M_PI), M_PI, EPSILON);
}

TEST_F(CommonFunctionsTest, AngleWrapNegativePi) {
    // -π wird zu π normalisiert
    EXPECT_NEAR(angleWrap(-M_PI), M_PI, EPSILON);
}

TEST_F(CommonFunctionsTest, AngleWrapGreaterThanPi) {
    // 3.5 rad > π → Normalisierung
    double result = angleWrap(3.5);
    EXPECT_GT(result, -M_PI);
    EXPECT_LE(result, M_PI);
    EXPECT_NEAR(result, 3.5 - 2.0 * M_PI, EPSILON);
}

TEST_F(CommonFunctionsTest, AngleWrapLessThanNegativePi) {
    // -4.0 rad < -π → Normalisierung
    double result = angleWrap(-4.0);
    EXPECT_GT(result, -M_PI);
    EXPECT_LE(result, M_PI);
}

TEST_F(CommonFunctionsTest, AngleWrapMultipleRotations) {
    // 10π → 0
    EXPECT_NEAR(angleWrap(10.0 * M_PI), 0.0, EPSILON);
}

// clamp Tests (3 Tests)
TEST_F(CommonFunctionsTest, ClampValueInRange) {
    EXPECT_NEAR(clamp(0.5, 0.0, 1.0), 0.5, EPSILON);
}

TEST_F(CommonFunctionsTest, ClampValueBelowMin) {
    EXPECT_NEAR(clamp(-0.5, 0.0, 1.0), 0.0, EPSILON);
}

TEST_F(CommonFunctionsTest, ClampValueAboveMax) {
    EXPECT_NEAR(clamp(1.5, 0.0, 1.0), 1.0, EPSILON);
}

// ============================================================================
// Test-Gruppe 2: PI-Regler (15 Tests)
// ============================================================================

class PIControllerTest : public ::testing::Test {
protected:
    PIParams params;
    PIState state;

    void SetUp() override {
        // Standard-Parameter
        params.Kp = 1.0;
        params.Ki = 1.5;
        params.v_min = 0.0;
        params.v_max = 1.5;

        state = init_pi();
    }
};

TEST_F(PIControllerTest, Initialization) {
    EXPECT_NEAR(state.v_cmd, 0.0, EPSILON);
    EXPECT_NEAR(state.e_pre, 0.0, EPSILON);
}

TEST_F(PIControllerTest, ZeroError) {
    // Keine Änderung bei Fehler = 0
    double v_cmd_before = state.v_cmd;
    double v_cmd = pi_step(params, state, 0.5, 0.5, 0.02);
    EXPECT_NEAR(v_cmd, v_cmd_before, 0.01);
}

TEST_F(PIControllerTest, PositiveErrorSmall) {
    // Kleiner positiver Fehler → Beschleunigung
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
    // Negativer Fehler → Verzögerung
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
    // Test Maximalgeschwindigkeitsbegrenzung
    for (int i = 0; i < 100; i++) {
        pi_step(params, state, 2.0, 0.0, 0.02);
    }
    EXPECT_LE(state.v_cmd, params.v_max);
}

TEST_F(PIControllerTest, MinVelocityLimit) {
    // Test Minimalgeschwindigkeitsbegrenzung
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
    // e_pre sollte aktualisiert werden
    EXPECT_NEAR(state.e_pre, v_ref - v_k, EPSILON);
}

TEST_F(PIControllerTest, ProportionalTerm) {
    // Nur P-Term testen (dt sehr klein)
    double v_cmd = pi_step(params, state, 0.5, 0.4, 0.001);
    EXPECT_GT(v_cmd, 0.0);
}

TEST_F(PIControllerTest, IntegralAccumulation) {
    // I-Term sollte über Zeit akkumulieren
    double v_cmd_1 = pi_step(params, state, 0.5, 0.4, 0.02);
    double v_cmd_2 = pi_step(params, state, 0.5, 0.4, 0.02);
    EXPECT_GT(v_cmd_2, v_cmd_1); // Integral wächst
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
    // Konvergenztest mit vereinfachtem Fahrzeugmodell
    double v_actual = 0.0;
    double v_ref = 0.5;
    double tau = 0.1; // Fahrzeugzeitkonstante
    double dt = 0.02;

    for (int i = 0; i < 100; i++) {
        double v_cmd = pi_step(params, state, v_ref, v_actual, dt);
        // Vereinfachte Fahrzeugdynamik: dv/dt = (v_cmd - v) / tau
        v_actual += (v_cmd - v_actual) / tau * dt;
    }

    // Sollte nahe Sollwert konvergieren
    EXPECT_NEAR(v_actual, v_ref, 0.15);
}

// ============================================================================
// Test-Gruppe 3: Motor Mapping (9 Tests)
// ============================================================================

class MotorMappingTest : public ::testing::Test {
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
    // Über v_max sollte auf 1.0 begrenzt werden
    EXPECT_NEAR(speed_to_motor_level(2.0, v_max), 1.0, EPSILON);
}

TEST_F(MotorMappingTest, NegativeSpeed) {
    // Negative Geschwindigkeit sollte auf 0 begrenzt werden
    EXPECT_NEAR(speed_to_motor_level(-0.5, v_max), 0.0, EPSILON);
}

TEST_F(MotorMappingTest, VerySmallSpeed) {
    double result = speed_to_motor_level(0.01, v_max);
    EXPECT_GE(result, 0.0);
    EXPECT_LE(result, 1.0);
}

TEST_F(MotorMappingTest, LinearityCheck) {
    // Linearität prüfen
    double v1 = 0.3;
    double v2 = 0.6;
    double m1 = speed_to_motor_level(v1, v_max);
    double m2 = speed_to_motor_level(v2, v_max);
    EXPECT_NEAR(m2, 2.0 * m1, EPSILON);
}

// ============================================================================
// Test-Gruppe 4: Lateral Controller (10 Tests)
// ============================================================================

class LateralControllerTest : public ::testing::Test {
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
    // Geschwindigkeit sollte nicht unter 0.1 fallen
    LateralController controller(0.05, L, L_h, kp, kd);
    EXPECT_GE(controller.getVelocity(), 0.1);
}

TEST_F(LateralControllerTest, ZeroError) {
    LateralController controller(v_init, L, L_h, kp, kd);
    double delta = controller.compute(0.0, 0.0);
    EXPECT_NEAR(delta, 0.0, EPSILON);
}

TEST_F(LateralControllerTest, PositiveLateralError) {
    // Positive Querabweichung → Linkslenken (negatives delta)
    LateralController controller(v_init, L, L_h, kp, kd);
    double delta = controller.compute(0.1, 0.0);
    EXPECT_LT(delta, 0.0);
}

TEST_F(LateralControllerTest, NegativeLateralError) {
    // Negative Querabweichung → Rechtslenken (positives delta)
    LateralController controller(v_init, L, L_h, kp, kd);
    double delta = controller.compute(-0.1, 0.0);
    EXPECT_GT(delta, 0.0);
}

TEST_F(LateralControllerTest, PositiveHeadingError) {
    LateralController controller(v_init, L, L_h, kp, kd);
    double delta = controller.compute(0.0, 0.2);
    EXPECT_LT(delta, 0.0);
}

TEST_F(LateralControllerTest, NegativeHeadingError) {
    LateralController controller(v_init, L, L_h, kp, kd);
    double delta = controller.compute(0.0, -0.2);
    EXPECT_GT(delta, 0.0);
}

TEST_F(LateralControllerTest, SteeringAngleLimit) {
    // Lenkwinkel sollte auf ±30° begrenzt sein
    LateralController controller(v_init, L, L_h, kp, kd);
    double max_steering = M_PI / 6.0;

    // Großer Fehler
    double delta = controller.compute(1.0, 1.0);
    EXPECT_GE(delta, -max_steering);
    EXPECT_LE(delta, max_steering);
}

TEST_F(LateralControllerTest, CombinedError) {
    // Beide Fehler in gleiche Richtung
    LateralController controller(v_init, L, L_h, kp, kd);
    double delta1 = controller.compute(0.1, 0.0);  // Nur lateral
    double delta2 = controller.compute(0.0, 0.1);  // Nur heading
    double delta3 = controller.compute(0.1, 0.1);  // Beide

    // Kombinierter Fehler sollte größere Reaktion erzeugen
    EXPECT_LT(delta3, delta1);
    EXPECT_LT(delta3, delta2);
}

// ============================================================================
// Hauptfunktion
// ============================================================================

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}