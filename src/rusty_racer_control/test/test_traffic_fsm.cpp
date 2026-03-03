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
#include "rusty_racer_control/traffic_fsm2.h"

// -- Helper functions --------------------------------------------------------

TrafficParams defaultParams()
{
  TrafficParams p;
  p.conf_th = 0.0f;
  p.v_default = 1.5f;
  p.v_speed30 = 1.2f;
  p.v_highway = 1.8f;
  p.v_yield = 0.5f;
  p.d_stop_trigger = 1.0f;
  p.d_yield_trigger = 1.0f;
  p.d_release = 3.0f;
  p.stop_hold_ms = 3000;
  p.on_count = 3;
  p.off_count = 3;
  p.start_off_count = 6;
  p.start_stop_lock_dist_m = 0.30f;
  p.same_sign_block_dist_m = 1.0f;
  return p;
}

CamFrame cam(SignType type, float dist)
{
  CamFrame f;
  if (type == SignType::Unknown) {return f;}
  CamDetection d;
  d.type = type;
  d.distance_m = dist;
  d.confidence = 1.0f;
  d.valid = true;
  f.dets.push_back(d);
  return f;
}

// -- sign_id mapping ---------------------------------------------------------

TEST(BasicTest, SignIdMapping) {
  EXPECT_EQ(signIdToType(0), SignType::HighwayEnd);     // end_express_way
  EXPECT_EQ(signIdToType(1), SignType::Speed30End);     // end_zone_speed_limit
  EXPECT_EQ(signIdToType(2), SignType::HighwayStart);   // start_express_way
  EXPECT_EQ(signIdToType(3), SignType::Speed30Start);   // start_zone_speed_limit
  EXPECT_EQ(signIdToType(4), SignType::Stop);           // stop_sign
  EXPECT_EQ(signIdToType(5), SignType::YieldSlow);      // yield
  EXPECT_EQ(signIdToType(99), SignType::Unknown);
}

// String overload mapping
TEST(BasicTest, SignIdStringMapping) {
  EXPECT_EQ(signIdToType(std::string("stop_sign")), SignType::Stop);
  EXPECT_EQ(signIdToType(std::string("start_zone_speed_limit")), SignType::Speed30Start);
  EXPECT_EQ(signIdToType(std::string("end_zone_speed_limit")), SignType::Speed30End);
  EXPECT_EQ(signIdToType(std::string("start_express_way")), SignType::HighwayStart);
  EXPECT_EQ(signIdToType(std::string("end_express_way")), SignType::HighwayEnd);
  EXPECT_EQ(signIdToType(std::string("yield")), SignType::YieldSlow);
  EXPECT_EQ(signIdToType(std::string("garbage")), SignType::Unknown);
  EXPECT_EQ(signIdToType(std::string("")), SignType::Unknown);
}

// MockMsg with int sign_id (for int-overload tests)
struct MockMsg { int32_t sign_id; float distance; };

TEST(BasicTest, ToCamFrameValidInt) {
  MockMsg msg{4, 1.5f};   // sign_id=4 → stop_sign → Stop
  auto f = toCamFrame(msg);
  ASSERT_EQ(f.dets.size(), 1u);
  EXPECT_EQ(f.dets[0].type, SignType::Stop);
  EXPECT_FLOAT_EQ(f.dets[0].distance_m, 1.5f);
  EXPECT_TRUE(f.dets[0].valid);
}

TEST(BasicTest, ToCamFrameInvalidInt) {
  EXPECT_EQ(toCamFrame(MockMsg{99, 1.5f}).dets.size(), 0u);  // Out-of-range → Unknown
  EXPECT_EQ(toCamFrame(MockMsg{4, 0.0f}).dets.size(), 0u);   // Zero distance
  EXPECT_EQ(toCamFrame(MockMsg{4, -1.f}).dets.size(), 0u);   // Negative distance
}

// MockStringMsg with string sign_id (matches actual TrafficSign.msg)
struct MockStringMsg { std::string sign_id; float distance; };

TEST(BasicTest, ToCamFrameValidString) {
  MockStringMsg msg{"stop_sign", 1.5f};
  auto f = toCamFrame(msg);
  ASSERT_EQ(f.dets.size(), 1u);
  EXPECT_EQ(f.dets[0].type, SignType::Stop);
  EXPECT_FLOAT_EQ(f.dets[0].distance_m, 1.5f);
  EXPECT_TRUE(f.dets[0].valid);
}

TEST(BasicTest, ToCamFrameStringAllTypes) {
  EXPECT_EQ(
    toCamFrame(MockStringMsg{"start_zone_speed_limit", 2.0f}).dets[0].type,
    SignType::Speed30Start);
  EXPECT_EQ(
    toCamFrame(MockStringMsg{"end_zone_speed_limit", 2.0f}).dets[0].type,
    SignType::Speed30End);
  EXPECT_EQ(
    toCamFrame(MockStringMsg{"start_express_way", 2.0f}).dets[0].type,
    SignType::HighwayStart);
  EXPECT_EQ(
    toCamFrame(MockStringMsg{"end_express_way", 2.0f}).dets[0].type,
    SignType::HighwayEnd);
  EXPECT_EQ(
    toCamFrame(MockStringMsg{"yield", 2.0f}).dets[0].type,
    SignType::YieldSlow);
}

TEST(BasicTest, ToCamFrameInvalidString) {
  EXPECT_EQ(toCamFrame(MockStringMsg{"garbage", 1.5f}).dets.size(), 0u);
  EXPECT_EQ(toCamFrame(MockStringMsg{"", 1.5f}).dets.size(), 0u);
  EXPECT_EQ(toCamFrame(MockStringMsg{"stop_sign", 0.0f}).dets.size(), 0u);
  EXPECT_EQ(toCamFrame(MockStringMsg{"stop_sign", -1.f}).dets.size(), 0u);
}

// -- Start gate --------------------------------------------------------------

class GateTest : public ::testing::Test
{
protected:
  void SetUp() override {fsm_.reset(); p_ = defaultParams();}
  TrafficFSM fsm_;
  TrafficParams p_;
};

TEST_F(GateTest, InitiallyLocked) {
  auto d = fsm_.step(cam(SignType::Unknown, 0), 20, p_);
  EXPECT_TRUE(d.must_stop);
  EXPECT_FLOAT_EQ(d.v_ref_mps, 0.0f);
}

TEST_F(GateTest, StopSignKeepsLocked) {
  for (int i = 0; i < 10; i++) {
    auto d = fsm_.step(cam(SignType::Stop, 0.25f), 20, p_);
    EXPECT_TRUE(d.must_stop);
  }
}

TEST_F(GateTest, UnlockAfterStopGone) {
  // Stop visible for several frames
  for (int i = 0; i < 5; i++) {
    fsm_.step(cam(SignType::Stop, 0.25f), 20, p_);
  }

  // Stop disappears, need start_off_count=6 frames to unlock
  for (int i = 0; i < 5; i++) {
    auto d = fsm_.step(cam(SignType::Unknown, 0), 20, p_);
    EXPECT_TRUE(d.must_stop);   // Still locked for first 5 frames
  }
  // 6th frame: unlock
  auto d = fsm_.step(cam(SignType::Unknown, 0), 20, p_);
  EXPECT_FALSE(d.must_stop);
  EXPECT_GT(d.v_ref_mps, 0.0f);
}

// -- Speed mode switching ----------------------------------------------------

class ModeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    fsm_.reset();
    p_ = defaultParams();
    // Unlock start gate
    for (int i = 0; i < 10; i++) {
      fsm_.step(cam(SignType::Unknown, 0), 20, p_);
    }
  }
  TrafficFSM fsm_;
  TrafficParams p_;
};

TEST_F(ModeTest, DefaultSpeed) {
  auto d = fsm_.step(cam(SignType::Unknown, 0), 20, p_);
  EXPECT_FLOAT_EQ(d.v_ref_mps, p_.v_default);
  EXPECT_FALSE(d.must_stop);
}

TEST_F(ModeTest, EnterSpeed30) {
  for (int i = 0; i < 3; i++) {
    fsm_.step(cam(SignType::Speed30Start, 2.0f), 20, p_);
  }
  auto d = fsm_.step(cam(SignType::Speed30Start, 2.0f), 20, p_);
  EXPECT_FLOAT_EQ(d.v_ref_mps, p_.v_speed30);
}

TEST_F(ModeTest, ExitSpeed30) {
  for (int i = 0; i < 5; i++) {
    fsm_.step(cam(SignType::Speed30Start, 2.0f), 20, p_);
  }
  for (int i = 0; i < 3; i++) {
    fsm_.step(cam(SignType::Speed30End, 2.0f), 20, p_);
  }
  auto d = fsm_.step(cam(SignType::Speed30End, 2.0f), 20, p_);
  EXPECT_FLOAT_EQ(d.v_ref_mps, p_.v_default);
}

TEST_F(ModeTest, EnterHighway) {
  for (int i = 0; i < 5; i++) {
    fsm_.step(cam(SignType::HighwayStart, 2.0f), 20, p_);
  }
  auto d = fsm_.step(cam(SignType::Unknown, 0), 20, p_);
  EXPECT_FLOAT_EQ(d.v_ref_mps, p_.v_highway);
}

// -- Action triggers ---------------------------------------------------------

class ActionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    fsm_.reset();
    p_ = defaultParams();
    // Unlock start gate
    for (int i = 0; i < 10; i++) {
      fsm_.step(cam(SignType::Unknown, 0), 20, p_);
    }
  }
  TrafficFSM fsm_;
  TrafficParams p_;
};

TEST_F(ActionTest, StopTrigger) {
  for (int i = 0; i < 3; i++) {
    fsm_.step(cam(SignType::Stop, 0.8f), 20, p_);
  }
  auto d = fsm_.step(cam(SignType::Stop, 0.8f), 20, p_);
  EXPECT_TRUE(d.must_stop);
  EXPECT_FLOAT_EQ(d.v_ref_mps, 0.0f);
}

TEST_F(ActionTest, StopFarAwayNoTrigger) {
  for (int i = 0; i < 5; i++) {
    auto d = fsm_.step(cam(SignType::Stop, 2.0f), 20, p_);
    EXPECT_FALSE(d.must_stop);
  }
}

TEST_F(ActionTest, StopHold3Seconds) {
  // Trigger Stop
  for (int i = 0; i < 5; i++) {
    fsm_.step(cam(SignType::Stop, 0.8f), 20, p_);
  }
  // Should hold for 2900ms (145 frames * 20ms)
  for (int i = 0; i < 145; i++) {
    auto d = fsm_.step(cam(SignType::Unknown, 0), 20, p_);
    EXPECT_TRUE(d.must_stop);
  }
  // Wait past 3000ms
  for (int i = 0; i < 15; i++) {
    fsm_.step(cam(SignType::Unknown, 0), 20, p_);
  }
  auto d = fsm_.step(cam(SignType::Unknown, 0), 20, p_);
  EXPECT_FALSE(d.must_stop);
}

TEST_F(ActionTest, YieldTrigger) {
  for (int i = 0; i < 4; i++) {
    fsm_.step(cam(SignType::YieldSlow, 0.8f), 20, p_);
  }
  auto d = fsm_.step(cam(SignType::YieldSlow, 0.8f), 20, p_);
  EXPECT_FALSE(d.must_stop);
  EXPECT_FLOAT_EQ(d.v_ref_mps, p_.v_yield);
}

// -- Debounce & one-shot -----------------------------------------------------

TEST(DebounceTest, FlickerRejected) {
  TrafficFSM fsm;
  fsm.reset();
  TrafficParams p = defaultParams();
  // Unlock gate
  for (int i = 0; i < 10; i++) {
    fsm.step(cam(SignType::Unknown, 0), 20, p);
  }
  // Flicker: appear-disappear
  fsm.step(cam(SignType::Stop, 0.8f), 20, p);
  fsm.step(cam(SignType::Unknown, 0), 20, p);
  fsm.step(cam(SignType::Stop, 0.8f), 20, p);
  auto d = fsm.step(cam(SignType::Stop, 0.8f), 20, p);
  EXPECT_FALSE(d.must_stop);   // Debounced, should not trigger
}

TEST(OneShotTest, NoDuplicateTriggerWithin1m) {
  TrafficFSM fsm;
  fsm.reset();
  TrafficParams p = defaultParams();
  // Unlock gate
  for (int i = 0; i < 10; i++) {
    fsm.step(cam(SignType::Unknown, 0), 20, p);
  }
  // First stop trigger
  for (int i = 0; i < 5; i++) {
    fsm.step(cam(SignType::Stop, 0.8f), 20, p);
  }
  // Wait for StopHold to end (3s)
  for (int i = 0; i < 160; i++) {
    fsm.step(cam(SignType::Unknown, 0), 20, p);
  }
  // Still within 1m: should NOT re-trigger
  for (int i = 0; i < 10; i++) {
    auto d = fsm.step(cam(SignType::Stop, 0.8f), 20, p);
    EXPECT_FALSE(d.must_stop);
  }
  // Move past d_release (3.0m) to clear cooldown
  for (int i = 0; i < 5; i++) {
    fsm.step(cam(SignType::Stop, 3.5f), 20, p);
  }
  // Approach again (0.8m): should trigger
  for (int i = 0; i < 5; i++) {
    fsm.step(cam(SignType::Stop, 0.8f), 20, p);
  }
  auto d = fsm.step(cam(SignType::Stop, 0.8f), 20, p);
  EXPECT_TRUE(d.must_stop);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
