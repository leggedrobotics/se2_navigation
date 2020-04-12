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

se2_planning::ReedsSheppState randomState(
    const se2_planning::OmplReedsSheppPlannerParameters &parameters)
{
  se2_planning::ReedsSheppState s;
  s.x_ = test::randomNumber(parameters.xLowerBound_, parameters.xUpperBound_);
  s.y_ = test::randomNumber(parameters.yLowerBound_, parameters.yUpperBound_);
  s.yaw_ = test::randomNumber(-M_PI, M_PI);
  return s;
}

TEST(Planning, OmplReedsSheppPlanner)
{
  const int seed = test::seedRndGenerator();
  const int testCases = 100;

  se2_planning::OmplReedsSheppPlannerParameters parameters;
  const double stateBound = 10.0;
  parameters.maxPlanningTime_ = 0.12;
  parameters.turningRadius_ = 1.0;
  parameters.xLowerBound_ = -stateBound * 1.5;
  parameters.xUpperBound_ = stateBound * 1.5;
  parameters.yLowerBound_ = -stateBound * 1.5;
  parameters.yUpperBound_ = stateBound * 1.5;
  parameters.plannerRange_ = 3.0*stateBound;

  se2_planning::OmplReedsSheppPlanner planner;
  planner.setParameters(parameters);
  planner.initialize();
  int sucesses = 0;
  for (int i = 0; i < testCases; ++i) {
    const auto start = randomState(parameters);
    const auto goal = randomState(parameters);
    planner.setStartingState(start);
    planner.setGoalState(goal);
    const bool status = planner.plan();
    EXPECT_TRUE(status);

    se2_planning::ReedsSheppPath path;
    planner.getPath(&path);
    const auto firstState = path.segment_.front().point_.front();
    const auto lastState = path.segment_.back().point_.back();
    EXPECT_TRUE(firstState == start);
    const bool reachedLastState = lastState == goal;
    EXPECT_TRUE(reachedLastState);
    if (reachedLastState){
      sucesses++;
    }
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test Planning, OmplReedsSheppPlanner failed with seed: " << seed << std::endl;
    std::cout << "Planning success rate: " << sucesses <<"/"<<testCases<<std::endl;
  }
}

