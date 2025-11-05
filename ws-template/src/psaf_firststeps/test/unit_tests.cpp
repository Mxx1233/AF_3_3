/**
 * @file unit_tests.cpp
 * @brief This file contains the unit tests for firststeps
 * @author PSAF
 * @data 2022-06-01
 * @todo implement unit tests
 */
#include "gtest/gtest.h"


TEST(FirstStepsSampleTest, Test1) {
  EXPECT_EQ(1, 1);
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
