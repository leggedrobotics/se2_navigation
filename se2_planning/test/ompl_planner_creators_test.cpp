/*
 * ompl_planner_creators_test.cpp
 *
 *  Created on: May 1, 2020
 *      Author: jelavice
 */




#include <gtest/gtest.h>
#include <cmath>

#include "se2_planning/ompl_planner_creators.hpp"
#include "test_helpers.hpp"

#include <gtest/gtest.h>
#include <cmath>

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "se2_planning/GridMapLazyStateValidator.hpp"
#include "se2_planning/OmplReedsSheppPlanner.hpp"
#include "test_helpers.hpp"

using namespace se2_planning_test;


TEST(Planning, PlanRRTstar)
{
  const int seed = 9873620;
  const int testCases = 10;
  rndGenerator.seed(seed);
  ompl::RNG::setSeed(seed);
  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_NONE);

  se2_planning::OmplReedsSheppPlannerParameters parameters =
      createRectangularStateSpaceWithDefaultParams(10.0);

  se2_planning::OmplReedsSheppPlanner planner;
  setupPlanner(parameters, &planner);
  auto rrtStarPlanner = se2_planning::createPlanner(planner.getSimpleSetup()->getSpaceInformation(), "RRTstar");
  planner.setOmplPlanner(rrtStarPlanner);
  for (int i = 0; i < testCases; ++i){
    EXPECT_TRUE(simplePlanBetweenRandomStartAndGoalTest(planner, parameters));
  }

}

TEST(Planning, PlanRRTsharp)
{
  const int seed = 173628;
  const int testCases = 10;
  rndGenerator.seed(seed);
  ompl::RNG::setSeed(seed);
  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_NONE);

  se2_planning::OmplReedsSheppPlannerParameters parameters =
      createRectangularStateSpaceWithDefaultParams(10.0);

  se2_planning::OmplReedsSheppPlanner planner;
  setupPlanner(parameters, &planner);
  auto rrtSharp = se2_planning::createPlanner(planner.getSimpleSetup()->getSpaceInformation(), "RRTsharp");
  planner.setOmplPlanner(rrtSharp);
  for (int i = 0; i < testCases; ++i){
    EXPECT_TRUE(simplePlanBetweenRandomStartAndGoalTest(planner, parameters));
  }

}

TEST(Planning, BITstar)
{
  const int seed = 173628;
  const int testCases = 10;
  rndGenerator.seed(seed);
  ompl::RNG::setSeed(seed);
  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_NONE);

  se2_planning::OmplReedsSheppPlannerParameters parameters =
      createRectangularStateSpaceWithDefaultParams(10.0);

  se2_planning::OmplReedsSheppPlanner planner;
  setupPlanner(parameters, &planner);
  auto bitStar = se2_planning::createPlanner(planner.getSimpleSetup()->getSpaceInformation(), "BITstar");
  planner.setOmplPlanner(bitStar);
  for (int i = 0; i < testCases; ++i){
    EXPECT_TRUE(simplePlanBetweenRandomStartAndGoalTest(planner, parameters));
  }

}
