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

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include "rusty_racer_control/lateral_controller.h"
#include "rusty_racer_control/laengsfuehrung_controller.h"
#include "rusty_racer_control/motor_mapping.h"

// -- Lateral controller (3-param PD, no curvature feedforward) ---------------

class LateralTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ctrl_ = std::make_unique<LateralController>(0.5, 0.257, 0.0, 0.8, 0.2);
  }
  std::unique_ptr<LateralController> ctrl_;
};

TEST_F(LateralTest, NoCorrectionWhenCentered) {
  EXPECT_NEAR(ctrl_->compute(0.0, 0.0, 0.0), 0.0, 0.01);
}

TEST_F(LateralTest, RightDeviationCorrection) {
  double delta = ctrl_->compute(0.1, 0.0, 0.0);
  EXPECT_LT(delta, 0.0);   // Right deviation -> steer left
}

TEST_F(LateralTest, LeftDeviationCorrection) {
  double delta = ctrl_->compute(-0.1, 0.0, 0.0);
  EXPECT_GT(delta, 0.0);   // Left deviation -> steer right
}

TEST_F(LateralTest, TargetOffsetTracking) {
  double delta = ctrl_->compute(0.0, 0.06, 0.0);
  EXPECT_GT(delta, 0.0);   // Target right offset -> steer right
}

TEST_F(LateralTest, HeadingErrorCorrection) {
  double delta = ctrl_->compute(0.0, 0.0, 0.1);
  EXPECT_LT(delta, 0.0);   // Positive heading error -> steer left
}

TEST_F(LateralTest, Saturation) {
  double delta = ctrl_->compute(1.0, 0.0, 0.5);
  // Saturation limit: atan(pi/6) ~ 0.4824
  const double sat = std::atan(M_PI / 6.0);
  EXPECT_GE(delta, -sat - 1e-6);
  EXPECT_LE(delta, sat + 1e-6);
}

TEST_F(LateralTest, OutputFinite) {
  ctrl_->updateVelocity(2.0);
  double delta = ctrl_->compute(0.3, 0.0, 0.2);
  EXPECT_TRUE(std::isfinite(delta));
}

// -- Longitudinal PI controller ----------------------------------------------

class PITest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    p_.Kp = 1.0;
    p_.Ki = 0.01;
    p_.v_min = 0.0;
    p_.v_max = 2.9;
    s_ = init_pi();
  }
  PIParams p_;
  PIState s_;
};

TEST_F(PITest, InitialState) {
  EXPECT_FLOAT_EQ(s_.v_cmd, 0.0);
  EXPECT_FLOAT_EQ(s_.e_pre, 0.0);
}

TEST_F(PITest, HoldSpeed) {
  // Incremental PI: v_cmd starts at 0, e_k=0 each step -> v_cmd stays 0
  for (int i = 0; i < 200; i++) {
    pi_step(p_, s_, 1.5, 1.5, 0.02);
  }
  double v = s_.v_cmd;
  EXPECT_GE(v, 0.0);
}

TEST_F(PITest, Accelerate) {
  double v1 = pi_step(p_, s_, 1.5, 1.0, 0.02);
  EXPECT_GT(v1, 0.0);
  double v2 = pi_step(p_, s_, 1.5, 1.0, 0.02);
  EXPECT_GE(v2, v1);   // Monotonically increasing
}

TEST_F(PITest, Decelerate) {
  double v = pi_step(p_, s_, 1.0, 1.5, 0.02);
  EXPECT_LT(v, 1.5);
}

TEST_F(PITest, MaxSaturation) {
  double v = pi_step(p_, s_, 5.0, 0.0, 0.02);
  EXPECT_LE(v, p_.v_max);
}

TEST_F(PITest, MinSaturation) {
  double v = pi_step(p_, s_, -1.0, 0.5, 0.02);
  EXPECT_GE(v, p_.v_min);
}

// -- Motor mapping -----------------------------------------------------------

TEST(MotorMappingTest, ZeroSpeed) {
  EXPECT_FLOAT_EQ(speed_to_motor_level(0.0, 2.9), 0.0);
}
TEST(MotorMappingTest, MaxSpeed) {
  EXPECT_FLOAT_EQ(speed_to_motor_level(2.9, 2.9), 1.0);
}
TEST(MotorMappingTest, HalfSpeed) {
  EXPECT_NEAR(speed_to_motor_level(1.45, 2.9), 0.5, 0.01);
}
TEST(MotorMappingTest, OverMax) {
  EXPECT_FLOAT_EQ(speed_to_motor_level(5.0, 2.9), 1.0);
}
TEST(MotorMappingTest, Negative) {
  EXPECT_FLOAT_EQ(speed_to_motor_level(-1.0, 2.9), 0.0);
}

TEST(MotorMappingTest, Linear) {
  double l1 = speed_to_motor_level(0.5, 2.9);
  double l2 = speed_to_motor_level(1.0, 2.9);
  double l3 = speed_to_motor_level(1.5, 2.9);
  EXPECT_GT(l2, l1);
  EXPECT_GT(l3, l2);
  EXPECT_NEAR(l2 - l1, l3 - l2, 0.01);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
