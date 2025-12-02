#include <gtest/gtest.h>
#include "low_level_pkg/state_estimation_core.hpp"

class StateEstimatorTest : public ::testing::Test {
protected:
  StateEstimatorCore::Config cfg;
  void SetUp() override
  {
    cfg.wheel_circumference = 0.22;
    cfg.direction_sign = 1;
    cfg.dt_timeout_sec = 0.5;
  }
};

TEST_F(StateEstimatorTest, VelocityCalculationCorrect) {
  StateEstimatorCore estimator(cfg);
  estimator.setDt8(1.0, 0.0);

  // v = (1/8 * 0.22) / 1.0 = 0.0275
  EXPECT_NEAR(estimator.getState().v_lin, 0.0275, 1e-5);
}

TEST_F(StateEstimatorTest, DirectionSignInverted) {
  cfg.direction_sign = -1;
  StateEstimatorCore estimator(cfg);

  estimator.setDt8(1.0, 0.0);
  EXPECT_NEAR(estimator.getState().v_lin, -0.0275, 1e-5);
}

TEST_F(StateEstimatorTest, ZeroOrNegativePeriodHandled) {
  StateEstimatorCore estimator(cfg);

  estimator.setDt8(0.0, 0.0);
  EXPECT_DOUBLE_EQ(estimator.getState().v_lin, 0.0);

  estimator.setDt8(-1.0, 0.0);
  EXPECT_DOUBLE_EQ(estimator.getState().v_lin, 0.0);
}

TEST_F(StateEstimatorTest, TimeoutStopsRobot) {
  StateEstimatorCore estimator(cfg);

  estimator.setDt8(0.1, 0.0);
  estimator.setImu(0.0);
  estimator.update(0.0);
  EXPECT_GT(estimator.getState().v_lin, 0.0);

  estimator.update(0.4);
  EXPECT_GT(estimator.getState().v_lin, 0.0);

  estimator.update(0.6);
  EXPECT_DOUBLE_EQ(estimator.getState().v_lin, 0.0);
}

TEST_F(StateEstimatorTest, IntegrationStraightLine) {
  StateEstimatorCore estimator(cfg);
  double p = (1.0 / 8.0) * 0.22;
  estimator.setDt8(p, 0.0);
  estimator.setImu(0.0);

  estimator.update(0.0);  // t0
  estimator.update(0.1);  // t1 (dt=1.0)

  auto s = estimator.getState();
  EXPECT_NEAR(s.x, 0.1, 1e-3);
  EXPECT_NEAR(s.y, 0.0, 1e-3);
  EXPECT_NEAR(s.yaw, 0.0, 1e-3);
}

TEST_F(StateEstimatorTest, YawNormalization) {
  StateEstimatorCore estimator(cfg);
  estimator.setDt8(1.0, 0.0);
  estimator.setImu(4.0);  // w = 4 rad/s

  estimator.update(0.0);
  estimator.update(1.0);

  EXPECT_LT(estimator.getState().yaw, M_PI);
  EXPECT_GT(estimator.getState().yaw, -M_PI);
  EXPECT_NEAR(estimator.getState().yaw, 4.0 - 2.0 * M_PI, 1e-3);
}
