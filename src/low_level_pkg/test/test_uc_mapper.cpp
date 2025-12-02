#include <gtest/gtest.h>
#include "low_level_pkg/uc_mapper.hpp"

class UcMapperTest : public ::testing::Test {
protected:
  UcMapper::Config cfg;
  const double DEG_25_RAD = M_PI / 180.0 * 25.0;

  void SetUp() override
  {
    cfg.angle_left_rad = -DEG_25_RAD;
    cfg.angle_right_rad = DEG_25_RAD;
    cfg.set_steering_left_max = -250;
    cfg.set_steering_right_max = 250;
    cfg.set_motor_level_forward_max = 1000;
    cfg.set_motor_level_backward_max = 500;
  }
};

// --- 1. Lenkung ---

// Test A: Zum Mittelpunkt drehen
TEST_F(UcMapperTest, SteeringCenter) {
    UcMapper mapper(cfg);
    auto output = mapper.mapCommand(0.0, 0.0);
    EXPECT_EQ(output.steering, 0);
}

// Test B: zur maximalen rechten Grenze drehen
TEST_F(UcMapperTest, SteeringMaxRightClamping) {
    UcMapper mapper(cfg);
    auto output = mapper.mapCommand(DEG_25_RAD + 0.1, 0.0);
    EXPECT_EQ(output.steering, 250);
}

// Test C: zur maximalen linken Grenze drehen
TEST_F(UcMapperTest, SteeringMaxLeftClamping) {
    UcMapper mapper(cfg);
    auto output = mapper.mapCommand(-DEG_25_RAD - 0.1, 0.0);
    EXPECT_EQ(output.steering, -250);
}

// Test D: Lineare Interpolation der Lenkung (halbe Rechtskurve)
TEST_F(UcMapperTest, SteeringLinearInterpolation) {
    UcMapper mapper(cfg);
    // 12.5 DEG (DEG_25_RAD / 2.0)
    auto output = mapper.mapCommand(DEG_25_RAD / 2.0, 0.0);
    // Erwartungswert：250 / 2 = 125
    EXPECT_EQ(output.steering, 125);
}

// --- 2. Motor-Mapping und Sicherheitsprüfungen ---

// Test E: Vorwärtsfahren (Granze)
TEST_F(UcMapperTest, FullThrottleForward) {
    UcMapper mapper(cfg);
    auto output = mapper.mapCommand(0.0, 1.0f);
    EXPECT_EQ(output.motor_fwd, 1000);
    EXPECT_EQ(output.motor_bwd, 0);
}

// Test F: Rückwärtsfahren (Granze)
TEST_F(UcMapperTest, FullThrottleBackward) {
    UcMapper mapper(cfg);
    auto output = mapper.mapCommand(0.0, -1.0f);
    EXPECT_EQ(output.motor_fwd, 0);
    EXPECT_EQ(output.motor_bwd, 500);
}

// Test G: Bremsen (Null)
TEST_F(UcMapperTest, StopCommand) {
    UcMapper mapper(cfg);
    auto output = mapper.mapCommand(0.0, 0.0f);
    EXPECT_EQ(output.motor_fwd, 0);
    EXPECT_EQ(output.motor_bwd, 0);
}

// Test H: Vorwärtsfahren(Halb)
TEST_F(UcMapperTest, HalfThrottleForward) {
    UcMapper mapper(cfg);
    auto output = mapper.mapCommand(0.0, 0.5f);
    // Erwartungswerte：1000 * 0.5 = 500
    EXPECT_EQ(output.motor_fwd, 500);
}

// Test I: Überschreitung
TEST_F(UcMapperTest, ThrottleOverBoundary) {
    UcMapper mapper(cfg);
    auto output = mapper.mapCommand(0.0, 2.0f);
    EXPECT_EQ(output.motor_fwd, 1000);

    output = mapper.mapCommand(0.0, -2.0f);
    EXPECT_EQ(output.motor_bwd, 500);
}
