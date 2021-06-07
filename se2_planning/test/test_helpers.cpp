/*
 * test_helpers.cpp
 *
 *  Created on: Apr 10, 2020
 *      Author: jelavice
 */

#include <grid_map_core/grid_map_core.hpp>
#include "test_helpers.hpp"
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <iostream>

namespace se2_planning_test {

std::mt19937 rndGenerator;

constexpr double piHalf = M_PI / 2.0;

int seedRndGenerator()
{
  const int seed = rand();
  rndGenerator.seed(seed);
  return seed;
}
double randomNumber(double lo, double hi)
{
  std::uniform_real_distribution<double> dist(lo, hi);
  return dist(rndGenerator);
}

void addObstacles(std::function<bool(double, double)> isObstacle, const std::string &layer,
                  grid_map::GridMap *gridMap)
{
  for (grid_map::GridMapIterator iterator(*gridMap); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position position;
    gridMap->getPosition(*iterator, position);
    if (isObstacle(position.x(), position.y())) {
      gridMap->at(layer, *iterator) = 1.0;  // obstacles
    }
  }
}

bool isInsideRectangle(double _x, double _y, double x0, double y0, double xLength, double yLength)
{
  const double halfSideLengthX = xLength / 2.0;
  const double halfSideLengthY = yLength / 2.0;
  const double x = _x - x0;
  const double y = _y - y0;
  return x < halfSideLengthX && x > -halfSideLengthX && y < halfSideLengthY && y > -halfSideLengthY;
}

se2_planning::SE2state randomState(const grid_map::GridMap &gm, double margin)
{
  se2_planning::SE2state s;
  const double l = gm.getLength()(0) / 2.0 - margin;
  const double w = gm.getLength()(1) / 2.0 - margin;
  s.x_ = randomNumber(-l, l);
  s.y_ = randomNumber(-w, w);
  s.yaw_ = randomNumber(-M_PI, M_PI);
  return s;
}

se2_planning::ReedsSheppState randomReedsSheppState(const grid_map::GridMap &gm, double margin)
{
  se2_planning::ReedsSheppState s;
  const double l = gm.getLength()(0) / 2.0 - margin;
  const double w = gm.getLength()(1) / 2.0 - margin;
  s.x_ = randomNumber(-l, l);
  s.y_ = randomNumber(-w, w);
  s.yaw_ = randomNumber(-M_PI, M_PI);
  return s;
}


grid_map::GridMap createGridMap(double length, double width, double resolution,
                                std::function<bool(double, double)> isAnObstacle)
{
  grid_map::GridMap gridMap;
  gridMap.setGeometry(grid_map::Length(length, width), resolution);
  gridMap.add(testLayer, 0.0);
  addObstacles(isAnObstacle, testLayer, &gridMap);
  return gridMap;
}

bool isPathCollisionFree(const se2_planning::ReedsSheppPath &path,
                         const se2_planning::StateValidator &validator)
{

  for (const auto &segment : path.segment_) {
    for (const auto &point : segment.point_) {
      if (!validator.isStateValid(point)) {
        return false;
      }
    }
  }
  return true;
}

bool isStartAndGoalStateOK(const se2_planning::ReedsSheppPath &path,
                           const se2_planning::ReedsSheppState &start,
                           const se2_planning::ReedsSheppState &goal)
{
  const auto firstState = path.segment_.front().point_.front();
  const auto lastState = path.segment_.back().point_.back();
  return firstState == start && lastState == goal;
}

void setCostThreshold(se2_planning::OmplReedsSheppPlanner *planner)
{
  auto si = planner->getSimpleSetup()->getSpaceInformation();
  ompl::base::OptimizationObjectivePtr optimizationObjective(
      std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));
  optimizationObjective->setCostThreshold(ompl::base::Cost(1e10));
  planner->getSimpleSetup()->setOptimizationObjective(optimizationObjective);
}

se2_planning::OmplReedsSheppPlannerParameters createDefaultParams(){
  se2_planning::OmplReedsSheppPlannerParameters params{};
  params.maxPlanningTime_ = 10.0;
  params.turningRadius_ = 1.0;
  params.boundariesMargin_ = 1.0;
  return params;
}

void setupPlanner(const se2_planning::OmplReedsSheppPlannerParameters &parameters,
                  se2_planning::OmplReedsSheppPlanner *planner)
{
  planner->setParameters(parameters);
  planner->initialize();
  setCostThreshold(planner);
}

bool simplePlanBetweenRandomStartAndGoalTest(
    se2_planning::OmplReedsSheppPlanner &planner,
    const se2_planning::OmplReedsSheppPlannerParameters &parameters, const grid_map::GridMap &map)
{
  const se2_planning::ReedsSheppState start = randomReedsSheppState(map, parameters.boundariesMargin_);
  const se2_planning::ReedsSheppState goal = randomReedsSheppState(map, parameters.boundariesMargin_);
  planner.setStartingState(start);
  planner.setGoalState(goal);
  const bool status = planner.plan();

  se2_planning::ReedsSheppPath path;
  planner.getPath(&path);
  const bool success = isStartAndGoalStateOK(path, start, goal);
  return status && success;
}

} /* namespace se2_planning_test */

