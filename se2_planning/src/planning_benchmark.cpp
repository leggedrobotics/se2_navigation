/*
 * planning_benchmark.cpp
 *
 *  Created on: Apr 26, 2020
 *      Author: jelavice
 */

#include <iostream>

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "se2_planning/GridMapLazyStateValidator.hpp"
#include "se2_planning/OmplReedsSheppPlanner.hpp"
#include "test_helpers.hpp"

#define duration(a) std::chrono::duration_cast<std::chrono::milliseconds>(a).count()

const int numTestCases = 500;
const std::string benchmarkLayer = "occupancy";

void setCostThreshold(se2_planning::OmplReedsSheppPlanner* planner) {
  auto si = planner->getSimpleSetup()->getSpaceInformation();
  ompl::base::OptimizationObjectivePtr optimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));
  optimizationObjective->setCostThreshold(ompl::base::Cost(1e10));
  planner->getSimpleSetup()->setOptimizationObjective(optimizationObjective);
}

void createRectangularStateSpace(double stateBound, se2_planning::OmplReedsSheppPlannerParameters* parameters) {
  parameters->xLowerBound_ = -stateBound;
  parameters->xUpperBound_ = stateBound;
  parameters->yLowerBound_ = -stateBound;
  parameters->yUpperBound_ = stateBound;
}

void runPlanner(se2_planning::OmplReedsSheppPlanner& planner) {
  const se2_planning::ReedsSheppState start(0.0, -10.0, 0.0);
  const se2_planning::ReedsSheppState goal(0.0, 10.0, 0.0);

  int count = 0;
  auto tStart = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < numTestCases; ++i) {
    planner.setStartingState(start);
    planner.setGoalState(goal);
    if (planner.plan()) {
      count++;
    }
  }
  auto tFinish = std::chrono::high_resolution_clock::now();
  std::cout << "Done, success rate: " << count << "/" << numTestCases << std::endl;
  std::cout << "total duration: " << duration(tFinish - tStart) << " miliseconds" << std::endl;
  std::cout << "per planning query: " << static_cast<double>(duration(tFinish - tStart)) / numTestCases << " miliseconds" << std::endl;
  std::cout << "\n===================\n";
}

void runNormalValidator(se2_planning::OmplReedsSheppPlanner& planner, const grid_map::GridMap& gridMap) {
  // create state validator
  auto plannerStateValidator =
      se2_planning::createGridMapStateValidator(gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), benchmarkLayer);
  planner.setStateValidator(std::move(plannerStateValidator));
  std::cout << "benchmarking standard validator " << std::endl;
  runPlanner(planner);
}

void runLazyValidator(se2_planning::OmplReedsSheppPlanner& planner, const grid_map::GridMap& gridMap) {
  // create state validator
  auto plannerStateValidator =
      se2_planning::createGridMapLazyStateValidator(gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), benchmarkLayer);
  planner.setStateValidator(std::move(plannerStateValidator));
  std::cout << "benchmarking lazy validator " << std::endl;
  runPlanner(planner);
}

void runLazyRandomValidator(se2_planning::OmplReedsSheppPlanner& planner, const grid_map::GridMap& gridMap) {
  // create state validator
  auto plannerStateValidator =
      se2_planning::createGridMapLazyStateValidator(gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), benchmarkLayer);
  plannerStateValidator->setIsUseRandomizedStrategy(true);
  planner.setStateValidator(std::move(plannerStateValidator));
  std::cout << "benchmarking lazy random validator " << std::endl;
  runPlanner(planner);
}

void runLazyRandomEarlyStoppingValidator(se2_planning::OmplReedsSheppPlanner& planner, const grid_map::GridMap& gridMap) {
  // create state validator
  auto plannerStateValidator =
      se2_planning::createGridMapLazyStateValidator(gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), benchmarkLayer);
  plannerStateValidator->setIsUseRandomizedStrategy(true);
  plannerStateValidator->setIsUseEarlyStoppingHeuristic(true);
  planner.setStateValidator(std::move(plannerStateValidator));
  std::cout << "benchmarking lazy early stopping random validator " << std::endl;
  runPlanner(planner);
}

int main(int argc, char** argv) {
  namespace test = se2_planning_test;
  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_NONE);
  const std::string testLayer = "occupancy";
  // create environment
  grid_map::GridMap gridMap;
  gridMap.setGeometry(grid_map::Length(40.0, 40.0), 0.1);
  gridMap.add(testLayer, 0.0);
  std::function<bool(double, double)> isAnObstacle = [](double x, double y) {
    return test::isInsideRectangle(x, y, -10.0, 0.0, 40.0, 2.0) || test::isInsideRectangle(x, y, 5.0, 5.0, 5.0, 5.0);
  };
  test::addObstacles(isAnObstacle, testLayer, &gridMap);

  // setup planner
  se2_planning::OmplReedsSheppPlannerParameters parameters;
  parameters.maxPlanningTime_ = 10.0;
  parameters.turningRadius_ = 1.0;
  createRectangularStateSpace(20.0, &parameters);
  se2_planning::OmplReedsSheppPlanner planner;
  planner.setParameters(parameters);
  planner.initialize();
  setCostThreshold(&planner);

  runNormalValidator(planner, gridMap);
  runLazyValidator(planner, gridMap);
  runLazyRandomValidator(planner, gridMap);
  runLazyRandomEarlyStoppingValidator(planner, gridMap);
  return 0;
}
