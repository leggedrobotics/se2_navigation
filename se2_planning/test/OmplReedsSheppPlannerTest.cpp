/*
 * OmplReedsSheppPlannerTest.cpp
 *
 *  Created on: Apr 10, 2020
 *      Author: jelavice
 */



#include <gtest/gtest.h>

// Math
#include <cmath>
#include "se2_planning/OmplReedsSheppPlanner.hpp"



template<typename T>
int toInt(T t)
{
  return static_cast<int>(t);
}

constexpr unsigned int numCasesPerTest = 2000;

TEST(Geoemtry, CircleValid)
{
  const int seed = ppt::seedRndGenerator();
  for (unsigned int i = 0; i < numCasesPerTest; ++i) {
    const auto circle = ppt::createRandomCircle();
    EXPECT_GE(circle.r_, 0.0);
    EXPECT_LE(std::fabs(circle.center_.x()), ppt::testPlaneWidth);
    EXPECT_LE(std::fabs(circle.center_.y()), ppt::testPlaneWidth);
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test Geometry, CircleValid failed with seed: " << seed << std::endl;
  }
}

