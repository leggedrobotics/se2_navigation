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
  auto si = planner.getSimpleSetup()->getSpaceInformation();
  ompl::base::OptimizationObjectivePtr optimizationObjective(
      std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));
  optimizationObjective->setCostThreshold(ompl::base::Cost(1e10));
  planner.getSimpleSetup()->setOptimizationObjective(optimizationObjective);

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
  for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position position;
    gridMap.getPosition(*iterator, position);
    const double sideLength = 5.0;
    const bool isInsideSquare = position.x() < sideLength / 2.0 && position.x() > -sideLength / 2.0
        && position.y() < sideLength / 2.0 && position.y() > -sideLength / 2.0;
    if (isInsideSquare) {
      gridMap.at(testLayer, *iterator) = 1.0;  // obstacles
    }
  }

  // stateValidator
  se2_planning::GridMapStateValidator validator;
  validator.setGridMap(gridMap);
  validator.setObstacleLayerName(testLayer);
  validator.setFootprint(se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5));
  std::unique_ptr<se2_planning::GridMapStateValidator> plannerValidator;
  plannerValidator = std::make_unique<se2_planning::GridMapStateValidator>();
  plannerValidator->setGridMap(gridMap);
  plannerValidator->setObstacleLayerName(testLayer);
  plannerValidator->setFootprint(se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5));

  se2_planning::OmplReedsSheppPlannerParameters parameters;
  parameters.maxPlanningTime_ = 10.0;
  parameters.turningRadius_ = 1.0;
  parameters.plannerRange_ = 15.0;
  createRectangularStateSpace(20.0, &parameters);

  se2_planning::OmplReedsSheppPlanner planner;
  planner.setParameters(parameters);
  planner.initialize();
  planner.setStateValidator(std::move(plannerValidator));
  auto si = planner.getSimpleSetup()->getSpaceInformation();
  ompl::base::OptimizationObjectivePtr optimizationObjective(
      std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));
  optimizationObjective->setCostThreshold(ompl::base::Cost(1e10));
  planner.getSimpleSetup()->setOptimizationObjective(optimizationObjective);

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

