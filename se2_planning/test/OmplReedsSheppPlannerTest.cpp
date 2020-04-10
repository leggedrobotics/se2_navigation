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
#include "test_helpers.hpp"

namespace test = se2_planning_test;

TEST(Planning, OmplReedsSheppPlanner)
{
  se2_planning::OmplReedsSheppPlannerParameters parameters;
  const int seed = test::seedRndGenerator();
  const int testCases = 10;



  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test Planning, OmplReedsSheppPlanner failed with seed: " << seed << std::endl;
  }
}

