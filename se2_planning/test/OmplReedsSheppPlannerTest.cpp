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
#include "se2_planning/GridMapStateValidator.hpp"
#include "se2_planning/OmplReedsSheppPlanner.hpp"
#include "test_helpers.hpp"

namespace test = se2_planning_test;
const std::string testLayer = "occupancy";

se2_planning::ReedsSheppState randomState(
    const se2_planning::OmplReedsSheppPlannerParameters &parameters)
{
  se2_planning::ReedsSheppState s;
  s.x_ = test::randomNumber(0.8 * parameters.xLowerBound_, 0.8 * parameters.xUpperBound_);
  s.y_ = test::randomNumber(0.8 * parameters.yLowerBound_, 0.8 * parameters.yUpperBound_);
  s.yaw_ = test::randomNumber(-M_PI, M_PI);
  return s;
}

void setCostThreshold(se2_planning::OmplReedsSheppPlanner *planner)
{
  auto si = planner->getSimpleSetup()->getSpaceInformation();
  ompl::base::OptimizationObjectivePtr optimizationObjective(
      std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));
  optimizationObjective->setCostThreshold(ompl::base::Cost(1e10));
  planner->getSimpleSetup()->setOptimizationObjective(optimizationObjective);
}

void createRectangularStateSpace(double stateBound,
                                 se2_planning::OmplReedsSheppPlannerParameters *parameters)
{
  parameters->xLowerBound_ = -stateBound;
  parameters->xUpperBound_ = stateBound;
  parameters->yLowerBound_ = -stateBound;
  parameters->yUpperBound_ = stateBound;
}

TEST(Planning, OmplReedsSheppPlanner)
{
  const int seed = test::seedRndGenerator();
  const int testCases = 100;
  ompl::RNG::setSeed(seed);
  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_NONE);
  se2_planning::OmplReedsSheppPlannerParameters parameters;
  parameters.maxPlanningTime_ = 10.0;
  parameters.turningRadius_ = 1.0;
  parameters.plannerRange_ = 10.0;
  createRectangularStateSpace(10.0, &parameters);

  se2_planning::OmplReedsSheppPlanner planner;
  planner.setParameters(parameters);
  planner.initialize();
  setCostThreshold(&planner);

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
    if (reachedLastState) {
      sucesses++;
    }
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test Planning, OmplReedsSheppPlanner failed with seed: " << seed << std::endl;
    std::cout << "Planning success rate: " << sucesses << "/" << testCases << std::endl;
  }
}

TEST(PlanningObstacle, OmplReedsSheppPlanner)
{
  const int seed = 1000;
  ompl::RNG::setSeed(seed);
  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_NONE);

  // create environment
  grid_map::GridMap gridMap;
  gridMap.setGeometry(grid_map::Length(40.0, 40.0), 0.1);
  gridMap.add(testLayer, 0.0);
  std::function<bool(double, double)> isAnObstacle = [](double x, double y) {
    return test::isInsideRectangle(x,y, 0.0, 0.0, 10.0, 10.0);
  };
  test::addObstacles(isAnObstacle, testLayer, &gridMap);

  // create state validator
  auto plannerStateValidator = se2_planning::createGridMapStateValidator(
      gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), testLayer);
  se2_planning::GridMapStateValidator validator = *plannerStateValidator;

  //setup planner
  se2_planning::OmplReedsSheppPlannerParameters parameters;
  parameters.maxPlanningTime_ = 10.0;
  parameters.turningRadius_ = 1.0;
  parameters.plannerRange_ = 15.0;
  createRectangularStateSpace(20.0, &parameters);
  se2_planning::OmplReedsSheppPlanner planner;
  planner.setParameters(parameters);
  planner.initialize();
  planner.setStateValidator(std::move(plannerStateValidator));
  setCostThreshold(&planner);

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
    const auto firstState = path.segment_.front().point_.front();
    const auto lastState = path.segment_.back().point_.back();
    EXPECT_TRUE(firstState == start);
    const bool reachedLastState = lastState == goal;
    EXPECT_TRUE(reachedLastState);

    //check whether the trajectory is truly collision free
    for (const auto &segment : path.segment_) {
      for (const auto &point : segment.point_) {
        EXPECT_TRUE(validator.isStateValid(point));
      }
    }
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test PlanningObstacle, OmplReedsSheppPlanner failed with seed: " << seed
              << std::endl;
  }
}

TEST(PlanningObstacle2, OmplReedsSheppPlanner)
{
  const int seed = 6786546;
  ompl::RNG::setSeed(seed);
  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_NONE);

  // create environment
  grid_map::GridMap gridMap;
  gridMap.setGeometry(grid_map::Length(40.0, 40.0), 0.1);
  gridMap.add(testLayer, 0.0);
  std::function<bool(double, double)> isAnObstacle =
      [](double x, double y) {
        return test::isInsideRectangle(x,y, -10.0, 0.0, 40.0,2.0) || test::isInsideRectangle(x,y, 5.0, 5.0, 5.0,5.0);
      };
  test::addObstacles(isAnObstacle, testLayer, &gridMap);

  // create state validator
  auto plannerStateValidator = se2_planning::createGridMapStateValidator(
      gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), testLayer);
  se2_planning::GridMapStateValidator validator = *plannerStateValidator;

  //setup planner
  se2_planning::OmplReedsSheppPlannerParameters parameters;
  parameters.maxPlanningTime_ = 10.0;
  parameters.turningRadius_ = 1.0;
  parameters.plannerRange_ = 15.0;
  createRectangularStateSpace(20.0, &parameters);
  se2_planning::OmplReedsSheppPlanner planner;
  planner.setParameters(parameters);
  planner.initialize();
  planner.setStateValidator(std::move(plannerStateValidator));
  setCostThreshold(&planner);

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
    const auto firstState = path.segment_.front().point_.front();
    const auto lastState = path.segment_.back().point_.back();
    EXPECT_TRUE(firstState == start);
    const bool reachedLastState = lastState == goal;
    EXPECT_TRUE(reachedLastState);

    //check whether the trajectory is truly collision free
    for (const auto &segment : path.segment_) {
      for (const auto &point : segment.point_) {
        EXPECT_TRUE(validator.isStateValid(point));
      }
    }
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test PlanningObstacle2, OmplReedsSheppPlanner failed with seed: " << seed
              << std::endl;
  }
}

