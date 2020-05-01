/*
 * OmplReedsSheppPlannerTest.cpp
 *
 *  Created on: Apr 10, 2020
 *      Author: jelavice
 */

#include <gtest/gtest.h>
#include <cmath>

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "se2_planning/GridMapLazyStateValidator.hpp"
#include "se2_planning/OmplReedsSheppPlanner.hpp"
#include "test_helpers.hpp"

using namespace se2_planning_test;


TEST(Planning, OmplReedsSheppPlanner)
{
  const int seed = 1321;
  const int testCases = 10;
  rndGenerator.seed(seed);
  ompl::RNG::setSeed(seed);
  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_NONE);

  se2_planning::OmplReedsSheppPlannerParameters parameters =
      createRectangularStateSpaceWithDefaultParams(10.0);

  se2_planning::OmplReedsSheppPlanner planner;
  setupPlanner(parameters, &planner);

  for (int i = 0; i < testCases; ++i){
    EXPECT_TRUE(simplePlanBetweenRandomStartAndGoalTest(planner, parameters));
  }

}

TEST(PlanningObstacle, PlanningWithStateValidator)
{
  const int seed = 1000;
  ompl::RNG::setSeed(seed);
  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_NONE);

  // create environment
  std::function<bool(double, double)> isAnObstacle = [](double x, double y) {
    return isInsideRectangle(x,y, 0.0, 0.0, 10.0, 10.0);
  };
  grid_map::GridMap gridMap = createGridMap(40.0, 40.0, 0.1, isAnObstacle);

  //setup planner
  se2_planning::OmplReedsSheppPlannerParameters parameters =
      createRectangularStateSpaceWithDefaultParams(20.0);
  se2_planning::OmplReedsSheppPlanner planner;
  setupPlanner(parameters, &planner);

  // create state validator
  auto plannerStateValidator = se2_planning::createGridMapStateValidator(
      gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), testLayer);
  se2_planning::GridMapStateValidator validator = *plannerStateValidator;
  planner.setStateValidator(std::move(plannerStateValidator));

  const se2_planning::ReedsSheppState start(0.0, -10.0, 0.0);
  const se2_planning::ReedsSheppState goal(0.0, 10.0, 0.0);

  // run 10 test cases
  for (int i = 0; i < 10; ++i) {
    planner.setStartingState(start);
    planner.setGoalState(goal);
    const bool status = planner.plan();
    EXPECT_TRUE(status);
    se2_planning::ReedsSheppPath path;
    planner.getPath(&path);
    EXPECT_TRUE(isStartAndGoalStateOK(path, start, goal));

    //check whether the trajectory is truly collision free
    EXPECT_TRUE(isPathCollisionFree(path, validator));
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test PlanningObstacle, PlanningWithStateValidator failed with seed: " << seed
              << std::endl;
  }
}

TEST(PlanningObstacle, PlanningWithStateValidatorHarder)
{
  const int seed = 6786546;
  ompl::RNG::setSeed(seed);
  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_NONE);

  // create environment
  std::function<bool(double, double)> isAnObstacle =
      [](double x, double y) {
        return isInsideRectangle(x,y, -10.0, 0.0, 40.0,2.0) || isInsideRectangle(x,y, 5.0, 5.0, 5.0,5.0);
      };
  grid_map::GridMap gridMap = createGridMap(40.0, 40.0, 0.1, isAnObstacle);

  //setup planner
  se2_planning::OmplReedsSheppPlannerParameters parameters =
      createRectangularStateSpaceWithDefaultParams(20.0);
  se2_planning::OmplReedsSheppPlanner planner;
  setupPlanner(parameters, &planner);

  // create state validator
  auto plannerStateValidator = se2_planning::createGridMapStateValidator(
      gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), testLayer);
  se2_planning::GridMapStateValidator validator = *plannerStateValidator;
  planner.setStateValidator(std::move(plannerStateValidator));

  const se2_planning::ReedsSheppState start(0.0, -10.0, 0.0);
  const se2_planning::ReedsSheppState goal(0.0, 10.0, 0.0);

  // run 10 test cases
  for (int i = 0; i < 10; ++i) {
    planner.setStartingState(start);
    planner.setGoalState(goal);
    const bool status = planner.plan();
    EXPECT_TRUE(status);
    se2_planning::ReedsSheppPath path;
    planner.getPath(&path);
    EXPECT_TRUE(isStartAndGoalStateOK(path, start, goal));

    //check whether the trajectory is truly collision free
    EXPECT_TRUE(isPathCollisionFree(path, validator));
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test PlanningObstacle, PlanningWithStateValidatorHarder failed with seed: " << seed
              << std::endl;
  }
}

TEST(PlanningObstacle, PlanningWithLazyStateValidator)
{
  const int seed = 1000;
  ompl::RNG::setSeed(seed);
  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_NONE);

  // create environment
  std::function<bool(double, double)> isAnObstacle = [](double x, double y) {
    return isInsideRectangle(x,y, 0.0, 0.0, 10.0, 10.0);
  };
  grid_map::GridMap gridMap = createGridMap(40.0, 40.0, 0.1, isAnObstacle);

  //setup planner
  se2_planning::OmplReedsSheppPlannerParameters parameters =
      createRectangularStateSpaceWithDefaultParams(20.0);
  se2_planning::OmplReedsSheppPlanner planner;
  setupPlanner(parameters, &planner);

  // create state validator
  auto plannerStateValidator = se2_planning::createGridMapLazyStateValidator(
      gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), testLayer);
  se2_planning::GridMapLazyStateValidator validator = *plannerStateValidator;
  planner.setStateValidator(std::move(plannerStateValidator));

  const se2_planning::ReedsSheppState start(0.0, -10.0, 0.0);
  const se2_planning::ReedsSheppState goal(0.0, 10.0, 0.0);

  auto runTest = [&](){
    for (int i = 0; i < 10; ++i) {
        planner.setStartingState(start);
        planner.setGoalState(goal);
        const bool status = planner.plan();
        EXPECT_TRUE(status);
        se2_planning::ReedsSheppPath path;
        planner.getPath(&path);
        EXPECT_TRUE(isStartAndGoalStateOK(path, start, goal));
        //check whether the trajectory is truly collision free
        EXPECT_TRUE(isPathCollisionFree(path, validator));
      }
  };

  runTest();

  plannerStateValidator = se2_planning::createGridMapLazyStateValidator(
      gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), testLayer);
  plannerStateValidator->setSeed(seed);
  plannerStateValidator->setIsUseEarlyStoppingHeuristic(true);
  planner.setStateValidator(std::move(plannerStateValidator));
  runTest();

  plannerStateValidator = se2_planning::createGridMapLazyStateValidator(
      gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), testLayer);
  plannerStateValidator->setSeed(seed);
  plannerStateValidator->setIsUseRandomizedStrategy(true);
  plannerStateValidator->setIsUseEarlyStoppingHeuristic(true);
  planner.setStateValidator(std::move(plannerStateValidator));
  runTest();

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test PlanningObstacle, PlanningWithLazyStateValidator failed with seed: " << seed
              << std::endl;
  }
}

TEST(PlanningObstacle, PlanningWithLazyStateValidatorHarder)
{
  const int seed = 6786546;
  ompl::RNG::setSeed(seed);
  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_NONE);

  // create environment
  std::function<bool(double, double)> isAnObstacle =
      [](double x, double y) {
        return isInsideRectangle(x,y, -10.0, 0.0, 40.0,2.0) || isInsideRectangle(x,y, 5.0, 5.0, 5.0,5.0);
      };
  grid_map::GridMap gridMap = createGridMap(40.0, 40.0, 0.1, isAnObstacle);

  //setup planner
  se2_planning::OmplReedsSheppPlannerParameters parameters =
      createRectangularStateSpaceWithDefaultParams(20.0);
  se2_planning::OmplReedsSheppPlanner planner;
  setupPlanner(parameters, &planner);

  // create state validator
  auto plannerStateValidator = se2_planning::createGridMapLazyStateValidator(
      gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), testLayer);
  se2_planning::GridMapLazyStateValidator validator = *plannerStateValidator;
  planner.setStateValidator(std::move(plannerStateValidator));

  const se2_planning::ReedsSheppState start(0.0, -10.0, 0.0);
  const se2_planning::ReedsSheppState goal(0.0, 10.0, 0.0);

  // run 10 test cases
  auto runTest = [&](){
      for (int i = 0; i < 10; ++i) {
          planner.setStartingState(start);
          planner.setGoalState(goal);
          const bool status = planner.plan();
          EXPECT_TRUE(status);
          se2_planning::ReedsSheppPath path;
          planner.getPath(&path);
          EXPECT_TRUE(isStartAndGoalStateOK(path, start, goal));
          //check whether the trajectory is truly collision free
          EXPECT_TRUE(isPathCollisionFree(path, validator));
        }
    };

    runTest();

    plannerStateValidator = se2_planning::createGridMapLazyStateValidator(
        gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), testLayer);
    plannerStateValidator->setSeed(seed);
    plannerStateValidator->setIsUseEarlyStoppingHeuristic(true);
    planner.setStateValidator(std::move(plannerStateValidator));
    runTest();

    plannerStateValidator = se2_planning::createGridMapLazyStateValidator(
        gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), testLayer);
    plannerStateValidator->setSeed(seed);
    plannerStateValidator->setIsUseRandomizedStrategy(true);
    plannerStateValidator->setIsUseEarlyStoppingHeuristic(true);
    planner.setStateValidator(std::move(plannerStateValidator));
    runTest();

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test PlanningObstacle, PlanningWithLazyStateValidatorHarder failed with seed: " << seed
              << std::endl;
  }
}

