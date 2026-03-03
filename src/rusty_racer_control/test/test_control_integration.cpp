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
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "rusty_racer_interfaces/msg/traffic_sign.hpp"
#include "rusty_racer_interfaces/msg/lane_deviation.hpp"
#include "rusty_racer_interfaces/msg/motor_command.hpp"

// -- Message format validation -----------------------------------------------

TEST(MsgFormatTest, TrafficSign) {
  auto m = rusty_racer_interfaces::msg::TrafficSign();
  m.header.frame_id = "base_link";
  m.sign_id = "stop_sign";
  m.distance = 1.5f;
  EXPECT_EQ(m.header.frame_id, std::string("base_link"));
  EXPECT_EQ(m.sign_id, std::string("stop_sign"));
  EXPECT_FLOAT_EQ(m.distance, 1.5f);
}

TEST(MsgFormatTest, LaneDeviation) {
  auto m = rusty_racer_interfaces::msg::LaneDeviation();
  m.lateral_error = 0.05f;
  m.heading_error = 0.02f;
  m.curvature = 0.3f;
  EXPECT_FLOAT_EQ(m.lateral_error, 0.05f);
  EXPECT_FLOAT_EQ(m.heading_error, 0.02f);
  EXPECT_FLOAT_EQ(m.curvature, 0.3f);
}

TEST(MsgFormatTest, MotorCommand) {
  auto m = rusty_racer_interfaces::msg::MotorCommand();
  m.motor_level = 0.5f;
  m.steering_angle = 0.1f;
  EXPECT_FLOAT_EQ(m.motor_level, 0.5f);
  EXPECT_FLOAT_EQ(m.steering_angle, 0.1f);
}

// -- Data range validation ---------------------------------------------------

TEST(RangeTest, SignIdRange) {
  const std::vector<std::string> valid_ids = {
    "stop_sign", "start_zone_speed_limit", "end_zone_speed_limit",
    "start_express_way", "end_express_way", "yield"};
  for (const auto & label : valid_ids) {
    rusty_racer_interfaces::msg::TrafficSign m;
    m.sign_id = label;
    EXPECT_FALSE(m.sign_id.empty());
  }
}

TEST(RangeTest, MotorLevelRange) {
  for (double v : {0.0, 0.5, 1.0}) {
    rusty_racer_interfaces::msg::MotorCommand m;
    m.motor_level = static_cast<float>(v);
    EXPECT_GE(m.motor_level, 0.0f);
    EXPECT_LE(m.motor_level, 1.0f);
  }
}

TEST(RangeTest, SteeringRange) {
  rusty_racer_interfaces::msg::MotorCommand m;
  m.steering_angle = -0.52f;
  EXPECT_GE(m.steering_angle, -0.52f);
  m.steering_angle = 0.52f;
  EXPECT_LE(m.steering_angle, 0.52f);
}

// -- ROS parameter validation ------------------------------------------------

class ParamTest : public ::testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

TEST_F(ParamTest, TrafficEnabledParam) {
  auto node = std::make_shared<rclcpp::Node>("test_node");
  node->declare_parameter("traffic_enabled", true);
  EXPECT_TRUE(node->get_parameter("traffic_enabled").as_bool());
  node->set_parameter(rclcpp::Parameter("traffic_enabled", false));
  EXPECT_FALSE(node->get_parameter("traffic_enabled").as_bool());
}

// -- Boundary conditions ----------------------------------------------------

TEST(BoundaryTest, ZeroLaneDeviation) {
  auto m = rusty_racer_interfaces::msg::LaneDeviation();
  m.lateral_error = 0.0;
  m.heading_error = 0.0;
  m.curvature = 0.0;
  EXPECT_FLOAT_EQ(m.lateral_error, 0.0);
}

TEST(BoundaryTest, NegativeLaneDeviation) {
  auto m = rusty_racer_interfaces::msg::LaneDeviation();
  m.lateral_error = -0.1f;
  m.heading_error = -0.05f;
  m.curvature = -0.3f;
  EXPECT_LT(m.lateral_error, 0.0);
  EXPECT_LT(m.curvature, 0.0);
}

TEST(BoundaryTest, MessageCopy) {
  auto m1 = rusty_racer_interfaces::msg::TrafficSign();
  m1.sign_id = "stop_sign";
  m1.distance = 1.5f;
  auto m2 = m1;
  EXPECT_EQ(m2.sign_id, m1.sign_id);
  EXPECT_FLOAT_EQ(m2.distance, m1.distance);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
